#include <atomic>
#include <cstdint>
#include <thread>
#include <array>
#include <mutex>
#include <iostream>
#include <spdlog/spdlog.h>
#include <boost/circular_buffer.hpp>
#include <immintrin.h>  // for _mm_pause()


// compile time constexpr and FPGA MMIO/DMA definitions
constexpr uint64_t MMIO_EVENT_REGISTER_ADDR = 0xFEDC0000;   // Simulated MMIO FPGA register
constexpr size_t DMA_BUFFER_SIZE = 1024;                    // Example DMA buffer size
constexpr size_t EVENT_BUFFER_SIZE = 256;                   // Circular buffer capacity in events
constexpr size_t CACHE_ALIGNMENT_SIZE = 64;                 // Cache alignment size in bytes

// dummy MMIO read (simulating hardware read)
inline uint32_t read_mmio_event() {
    // this dereferences the memory address of the MMIO event register
    // and uses volatile to force reevaluation every time, and casts the
    // dereferenced data at the address as a uint32_t
    
    // THIS IS WHAT WE WOULD DO IN REALITY
    // return *reinterpret_cast<volatile uint32_t*>(MMIO_EVENT_REGISTER_ADDR);

    // THIS IS WHAT WE WILL DO FOR TESTING
    static std::atomic<uint32_t> fake_event{0};
    static int counter = 0;
    
    // Simulate event every 1000 iterations
    if (++counter % 500 == 0) {
        fake_event.store(1);
    }

    return fake_event.exchange(0);
}

// dummy DMA buffer, aligned for cache performance
alignas(CACHE_ALIGNMENT_SIZE) std::array<uint32_t, DMA_BUFFER_SIZE> dma_buffer;


// DMA event struct
struct DMAEvent {
    uint32_t event_type;
    uint32_t dma_offset;    // offset into DMA buffer - in reality the correct DMA offset is specified by the FPGA
};

typedef boost::circular_buffer<DMAEvent> dma_event_buffer;

// MMIO producer function for thread
void mmio_event_producer(std::atomic_bool& running,
                         dma_event_buffer& event_buffer,
                         std::mutex& buffer_mutex,
                         std::atomic_bool& producer_done) {
    
    // load without CPU memory fences for optimised atomicity
    while (running.load(std::memory_order_relaxed)) {
        uint32_t event = read_mmio_event();
        if (event != 0) {
            // simulate the event recieved from the FPGA's memory mapped IO (MMIO) PCIe register address
            DMAEvent dma_evt{
                event,  // event_type
                0       // dma_offset index ino the DMA buffer to start reading data
            };

            // lock boost circular buffer and push
            std::unique_lock<std::mutex> lock(buffer_mutex);
            if (event_buffer.full()) {
                spdlog::error("Event buffer overflow, event lost {}", dma_evt.event_type);
            } else {
                event_buffer.push_back(dma_evt);
            }
        } else {
            _mm_pause();    // introduce a spin wait by telling the CPU to pause. Prevents speculative execution. CPU platform specific.
        }
    }
    // release to synchronise at this point
    producer_done.store(true, std::memory_order_release);
}

void dma_event_consumer(std::atomic_bool& running,
                        dma_event_buffer& event_buffer,
                        std::mutex& buffer_mutex,
                        std::atomic_bool& producer_done,
                        std::atomic_bool& consumer_done) {
    
    // load without CPU memory fences for optimised atomicity
    while (running.load(std::memory_order_relaxed) || !producer_done.load()) {
        DMAEvent evt;
        bool has_event = false;
        
        // we create a sub-scope because we don't need the lock for the entire scope, so we can std::lock_guard
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            if (!event_buffer.empty()) {
                evt = event_buffer.front();
                event_buffer.pop_front();
                has_event = true;
            }
        }

        if (has_event) {
            // get the data from the DMA offset specified by the event struct
            uint32_t data = dma_buffer[evt.dma_offset];
            spdlog::info("Processing DMA event: {}, data: {}", evt.event_type, data);
        } else {
            _mm_pause();    // introduce a spin wait by telling the CPU to pause. Prevents speculative execution. CPU platform specific.
        }
    }
    // release to synchronise at this point
    consumer_done.store(true, std::memory_order_release);
}

// shutdown handler
void signal_handler(std::atomic_bool& running) {
    running.store(false, std::memory_order_release);
}

int main() {
    // atomic flags for clean shutdown
    std::atomic_bool running{true};
    std::atomic_bool producer_done{false};
    std::atomic_bool consumer_done{false};

    // debug check
    spdlog::info("Main thread: running={}", running.load());

    // circular buffer for SCSP queue, can only handle 256 events, each at 64 bytes
    dma_event_buffer event_buffer(EVENT_BUFFER_SIZE);
    // mutex for boost::circular_buffer, apparently not lock free! How can we improve this?
    std::mutex buffer_mutex;

    // we pass using std::ref because std::thread copies by default, and we want to pass references, not copies
    std::thread producer_thread(mmio_event_producer,
                                std::ref(running),
                                std::ref(event_buffer),
                                std::ref(buffer_mutex),
                                std::ref(producer_done));
    
    std::thread consumer_thread(dma_event_consumer,
                                std::ref(running),
                                std::ref(event_buffer),
                                std::ref(buffer_mutex),
                                std::ref(producer_done),
                                std::ref(consumer_done));

    // simulate system running
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // simulate shutdown to end running after 10s
    signal_handler(running);

    // busy-poll wait for graceful shutdown
    while (!producer_done.load() || !consumer_done.load()) {
        _mm_pause();    // tell the CPU to wait, to not thrash on a busy poll loop
    }

    producer_thread.join();
    consumer_thread.join();

    spdlog::info("Shutdown completed gracefully");
    std::exit(EXIT_SUCCESS);
}