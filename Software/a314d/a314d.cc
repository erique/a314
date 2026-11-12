/*
 * Copyright (c) 2018-2023 Niklas Ekström
 */

#include <arpa/inet.h>

#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <linux/gpio.h>

#include <netinet/in.h>
#include <netinet/tcp.h>

#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <list>
#include <map>
#include <string>
#include <vector>

#define COUNT_DEFINED ( \
    (defined MODEL_TD ? 1 : 0) + \
    (defined MODEL_FE ? 1 : 0) + \
    (defined MODEL_CP ? 1 : 0) + \
    (defined MODEL_TF ? 1 : 0) \
)

#if COUNT_DEFINED == 0
#error Need to define MODEL_TD, MODEL_FE, MODEL_CP or MODEL_TF
#elif COUNT_DEFINED > 1
#error The MODEL_XX flags cannot be combined
#endif

#define LOGLEVEL_TRACE      10
#define LOGLEVEL_DEBUG      20
#define LOGLEVEL_INFO       30
#define LOGLEVEL_WARNING    40
#define LOGLEVEL_ERROR      50

static int loglevel = LOGLEVEL_INFO;

#define A314D_VERSION "1.1 (11 Nov 2026)"

#define logger_trace(...) do { if (loglevel <= LOGLEVEL_TRACE) fprintf(stdout, __VA_ARGS__); } while (0)
#define logger_debug(...) do { if (loglevel <= LOGLEVEL_DEBUG) fprintf(stdout, __VA_ARGS__); } while (0)
#define logger_info(...) do { if (loglevel <= LOGLEVEL_INFO) fprintf(stdout, __VA_ARGS__); } while (0)
#define logger_warning(...) do { if (loglevel <= LOGLEVEL_WARNING) fprintf(stdout, __VA_ARGS__); } while (0)
#define logger_error(...) do { if (loglevel <= LOGLEVEL_ERROR) fprintf(stderr, __VA_ARGS__); } while (0)

// Packets that are communicated across physical channels (A2R and R2A).
#define PKT_CONNECT             4
#define PKT_CONNECT_RESPONSE    5
#define PKT_DATA                6
#define PKT_EOS                 7
#define PKT_RESET               8

// Valid responses for PKT_CONNECT_RESPONSE.
#define CONNECT_OK              0
#define CONNECT_UNKNOWN_SERVICE 3

// Messages that are communicated between driver and client.
#define MSG_REGISTER_REQ        1
#define MSG_REGISTER_RES        2
#define MSG_DEREGISTER_REQ      3
#define MSG_DEREGISTER_RES      4
#define MSG_READ_MEM_REQ        5
#define MSG_READ_MEM_RES        6
#define MSG_WRITE_MEM_REQ       7
#define MSG_WRITE_MEM_RES       8
#define MSG_CONNECT             9
#define MSG_CONNECT_RESPONSE    10
#define MSG_DATA                11
#define MSG_EOS                 12
#define MSG_RESET               13

#define MSG_SUCCESS             1
#define MSG_FAIL                0

#define R2A_TAIL_UPDATED        1
#define A2R_HEAD_UPDATED        2

// TODO: These constants should be the same for both TD/FE and CP.
// Need to update a314.device in order to change these.
#if defined(MODEL_TD) || defined(MODEL_TF)
// Offset relative to communication area for queue pointers.
#define A2R_TAIL_OFFSET         0
#define R2A_HEAD_OFFSET         1
#define R2A_TAIL_OFFSET         2
#define A2R_HEAD_OFFSET         3

// Addresses of fixed data structures in shared memory.
#define CAP_BASE                0
#define A2R_BASE                4
#define R2A_BASE                260
#elif defined(MODEL_FE)
// Offset relative to communication area for queue pointers.
#define A2R_TAIL_OFFSET         0
#define R2A_HEAD_OFFSET         1
#define R2A_TAIL_OFFSET         2
#define A2R_HEAD_OFFSET         3

// Addresses of fixed data structures in shared memory.
#define HANDSHAKE_BASE          0
#define CAP_BASE                4
#define A2R_BASE                8
#define R2A_BASE                264
#elif defined(MODEL_CP)
// Offset relative to communication area for queue pointers.
#define R2A_TAIL_OFFSET         0
#define A2R_HEAD_OFFSET         1
#define A2R_TAIL_OFFSET         2
#define R2A_HEAD_OFFSET         3

// Addresses of fixed data structures in shared memory.
#define A2R_BASE                0
#define R2A_BASE                256
#define CAP_BASE                512
#endif

#if defined(MODEL_TD)
#define IRQ_GPIO                25
#define IRQ_GPIO_EDGE           GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING
#elif defined(MODEL_FE)
#define IRQ_GPIO                23
#define IRQ_GPIO_EDGE           GPIO_V2_LINE_FLAG_EDGE_RISING
#elif defined(MODEL_CP)
#define IRQ_GPIO                2
#define IRQ_GPIO_EDGE           GPIO_V2_LINE_FLAG_EDGE_RISING
#elif defined(MODEL_TF)
#define IRQ_GPIO                8
#define IRQ_GPIO_EDGE           GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING
#endif

#if defined(MODEL_TD)
// SPI commands.
#define READ_SRAM_CMD           0
#define WRITE_SRAM_CMD          1
#define READ_CMEM_CMD           2
#define WRITE_CMEM_CMD          3
#define SPI_PROTO_VER_CMD       255

#define SPI_PROTO_HDR_LEN       1
#define READ_SRAM_HDR_LEN       4

// Addresses to variables in CMEM.
#define BASE_ADDRESS_LEN        6
#define R_EVENTS_ADDRESS        12
#define R_ENABLE_ADDRESS        13
#define A_EVENTS_ADDRESS        14
#define A_ENABLE_ADDRESS        15

// Events that are communicated via IRQ from Amiga to Raspberry.
#define R_EVENT_A2R_TAIL        1
#define R_EVENT_R2A_HEAD        2
#define R_EVENT_BASE_ADDRESS    4

// Events that are communicated from Raspberry to Amiga.
#define A_EVENT_R2A_TAIL        1
#define A_EVENT_A2R_HEAD        2

#elif defined(MODEL_FE)
#define PIN_D(x)                (4 + x)
#define PIN_WR                  20
#define PIN_REQ                 21
#define PIN_ACK                 22
#define PIN_IRQ                 23
#define PIN_A(x)                (24 + x)

#define GPIO_DIR_0_MASK         0x3ffff000
#define GPIO_DIR_0_DIN          0x00000000
#define GPIO_DIR_0_DOUT         0x09249000
#define GPIO_DIR_1_MASK         0x3fffffff
#define GPIO_DIR_1_DIN          0x00000000
#define GPIO_DIR_1_DOUT         0x09249249
#define GPIO_DIR_2_MASK         0x00ffffff
#define GPIO_DIR_2              0x00249009

#define GPIO_PULL_NONE          0
#define GPIO_PULL_DOWN          1
#define GPIO_PULL_UP            2

#define REG_SRAM_BYTE           0
#define REG_SRAM_WORD           1
#define REG_ADDR_LO             2
#define REG_ADDR_HI             3
#define REG_INT_REQ             4
#define REG_INT_ENA             5
#define REG_CA_BASE_ADDR        6

#define IRQ_SET                 0x8000
#define IRQ_CLR                 0x0000
#define IRQ_R2A_TAIL            0x0001
#define IRQ_A2R_HEAD            0x0002
#define IRQ_A2R_TAIL            0x0004
#define IRQ_R2A_HEAD            0x0008
#define IRQ_BASE_ADDRESS        0x0010

#elif defined(MODEL_CP)
#define PIN_IRQ                 2
#define PIN_CLK                 4
#define PIN_REQ                 14
#define PIN_ACK                 15
#define PIN_D(x)                (16 + x)
#define PIN_A(x)                (24 + x)
#define PIN_WR                  27

#define PINS_OUT            ((0xff << PIN_D(0)) | (3 << PIN_A(0)) | (1 << PIN_WR) | (1 << PIN_REQ))
#define PINS_OUT_NOT_WR     ((0xff << PIN_D(0)) | (3 << PIN_A(0)) | (1 << PIN_REQ))

#define GPIO_DIR_0              0x08004000
#define GPIO_DIR_0_MASK         0x3f0071c0
#define GPIO_DIR_1_DIN          0x00001009
#define GPIO_DIR_1_DOUT         0x09241009
#define GPIO_DIR_2_DIN          0x08209000
#define GPIO_DIR_2_DOUT         0x08209249

#define GPIO_PULL_NONE          0
#define GPIO_PULL_DOWN          1
#define GPIO_PULL_UP            2

#define REG_SRAM                0
#define REG_IRQ                 1
#define REG_ADDR_LO             2
#define REG_ADDR_HI             3

#define REG_IRQ_SET             0x80
#define REG_IRQ_CLR             0x00
#define REG_IRQ_PI              0x02
#define REG_IRQ_CP              0x01

#elif defined(MODEL_TF)

// SPI commands.
#define READ_SRAM_CMD           0
#define WRITE_SRAM_CMD          1
#define READ_SINT_CMD           2
#define WRITE_SINT_CMD          3
#define SPI_PROTO_VER_CMD       255

#define SPI_PROTO_HDR_LEN       3           // 1 cmd byte + 2 delay
#define READ_SRAM_HDR_LEN       5           // 3 cmd bytes + 2 delay
#define READ_SINT_HDR_LEN       3           // 1 cmd byte + 2 delay

// Bit 7 Set/Clear on write 
// Bit 6 Amiga Interrupts Enabled (INT2). 1 = Enabled, 0 = Disabled
// Bit 5 RPi (Remote Interrupt). 1 = Pending, 0 = Not Pending (Amiga Can set but not Clear, RPi Can clear but not set)
// Bit 4 Amiga Interrupt.  1 = Pending, 0 = Not Pending (Amiga can clear but not set, RPi Can set but not clear)
// Bit 3 Unused
// Bit 2 Unused
// Bit 1 Unused
// Bit 0 Reset Event 

#define REG_IRQ_SET             0x80
#define REG_IRQ_CLR             0x00
#define REG_IRQ_INTENA          0x40
#define REG_IRQ_RPI             0x20
#define REG_IRQ_AMIGA           0x10
#define REG_IRQ_RESET           0x01
#endif

// Global variables.
static sigset_t original_sigset;

static uint8_t tx_buf[65536];
static uint8_t rx_buf[65536];

static volatile unsigned int *gpio;

#if defined(MODEL_TD) || defined(MODEL_TF)
static uint8_t mode = SPI_CS_HIGH;
static uint8_t bits = 8;
static uint32_t speed = 67000000;

static int spi_fd = -1;
static int spi_proto_ver = 0;
#elif defined(MODEL_FE)
static unsigned int current_address;
static int current_dir = 0; // 0 = input, 1 = output.
#elif defined(MODEL_CP)
static unsigned short current_address;
static int current_dir = 0; // 0 = input, 1 = output.

static bool read_a314_magic = false;
static int restart_counter;
#endif

static int gpio_irq_fd = -1;

static bool auto_clear_irq = false;
static time_t auto_clear_irq_after;

static int server_socket = -1;

static int epfd = -1;

static bool have_base_address = false;
static unsigned int base_address = 0;

#if defined(MODEL_TD) || defined(MODEL_FE)
#define BASE_ADDRESS base_address
#elif defined(MODEL_CP) || defined(MODEL_TF)
#define BASE_ADDRESS 0
#endif

static uint8_t channel_status[4];
static uint8_t channel_status_updated = 0;

static uint8_t recv_buf[256];
static uint8_t send_buf[256];

struct LogicalChannel;
struct ClientConnection;

#pragma pack(push, 1)
struct MessageHeader
{
    uint32_t length;
    uint32_t stream_id;
    uint8_t type;
}; //} __attribute__((packed));
#pragma pack(pop)

struct MessageBuffer
{
    int pos;
    std::vector<uint8_t> data;
};

struct RegisteredService
{
    std::string name;
    ClientConnection *cc;
};

struct PacketBuffer
{
    int type;
    std::vector<uint8_t> data;
};

struct ClientConnection
{
    int fd;

    int next_stream_id;

    int bytes_read;
    MessageHeader header;
    std::vector<uint8_t> payload;

    std::list<MessageBuffer> message_queue;

    std::list<LogicalChannel*> associations;
};

struct LogicalChannel
{
    int channel_id;

    ClientConnection *association;
    int stream_id;

    bool got_eos_from_ami;
    bool got_eos_from_client;

    std::list<PacketBuffer> packet_queue;
};

static void remove_association(LogicalChannel *ch);
static void close_all_logical_channels();
static void clear_packet_queue(LogicalChannel *ch);
static void create_and_enqueue_packet(LogicalChannel *ch, uint8_t type, uint8_t *data, uint8_t length);

static std::list<ClientConnection> connections;

// Set when an SPI transfer fails (spi_transfer()); cleared by the code that
// checks it.
static bool spi_error = false;

// The longest message a client may send (a memory write is the longest).
#define MAX_CLIENT_MSG_LEN (1024 * 1024)

// A memory request that is malformed, or whose SPI transfer failed, is not
// answered: the client's connection is closed instead
// (handle_client_connection_event()). That resets
// its streams on the Amiga, and no client ever gets a reply for a transfer
// that did not happen. A standalone a314fs.py registers again.
static ClientConnection *failed_cc = nullptr;

// An a2r head that was taken (its packets handled) but could not be written to
// SRAM, or -1. read_channel_status() keeps it instead of the head in SRAM, so
// those packets are not handled twice.
static int a2r_head_unwritten = -1;
static std::list<RegisteredService> services;
static std::list<LogicalChannel> channels;
static std::list<LogicalChannel*> send_queue;

struct OnDemandStart
{
    std::string service_name;
    std::string program;
    std::vector<std::string> arguments;
};

std::vector<OnDemandStart> on_demand_services;

static void load_config_file(const char *filename)
{
    FILE *f = fopen(filename, "rt");
    if (f == nullptr)
        return;

    char line[256];
    std::vector<char *> parts;

    while (fgets(line, 256, f) != nullptr)
    {
        char org_line[256];
        strcpy(org_line, line);

        bool in_quotes = false;

        int start = 0;
        for (int i = 0; i < 256; i++)
        {
            if (line[i] == 0)
            {
                if (start < i)
                    parts.push_back(&line[start]);
                break;
            }
            else if (line[i] == '"')
            {
                line[i] = 0;
                if (in_quotes)
                    parts.push_back(&line[start]);
                in_quotes = !in_quotes;
                start = i + 1;
            }
            else if (isspace(line[i]) && !in_quotes)
            {
                line[i] = 0;
                if (start < i)
                    parts.push_back(&line[start]);
                start = i + 1;
            }
        }

        if (parts.size() >= 2)
        {
            on_demand_services.emplace_back();
            auto &e = on_demand_services.back();
            e.service_name = parts[0];
            e.program = parts[1];
            for (int i = 1; i < parts.size(); i++)
                e.arguments.push_back(std::string(parts[i]));
        }
        else if (parts.size() != 0)
            logger_warning("Invalid number of columns in configuration file line: %s\n", org_line);

        parts.clear();
    }

    fclose(f);

    if (on_demand_services.empty())
        logger_warning("No registered services\n");
}

#if defined(MODEL_TD) || defined(MODEL_TF)

static int init_spi()
{
    spi_fd = open("/dev/spidev0.0", O_RDWR | O_CLOEXEC);
    if (spi_fd < 0)
        return -1;

    int ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ret |= ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ret |= ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret != 0)
        return ret;

    return 0;
}

static void shutdown_spi()
{
    if (spi_fd != -1)
        close(spi_fd);
}

static int check_spidev_bufsiz()
{
    int fd = open("/sys/module/spidev/parameters/bufsiz", O_RDONLY);
    if (fd == -1)
        return -1;

    char buf[32];
    auto actual = read(fd, buf, strlen(buf));

    close(fd);

    if (strncmp(buf, "65536", actual) != 0)
        return -2;

    return 0;
}

#if defined(MODEL_TF)
// CE0 is the SPI flash's chip select, on the same SPI bus as the A314. It is
// asserted while the Amiga reads the flash. An SPI transfer made then reads or
// writes garbage without any ioctl() error.
//
// With the FPGA side (tf4060v2 #123) the bus is arbitrated: the FPGA takes it
// only while GPIO7 (the A314 select) is idle, holds CE0 low while it has it,
// and ignores GPIO7 meanwhile. So before every transfer: wait for CE0 to be
// released, assert GPIO7, wait a moment (longer than the FPGA's input
// synchroniser and its grant-to-CE0 delay) and check CE0 again: if the Amiga
// got the bus in that same instant, release GPIO7 and wait for it. This needs
// the spi-a314 overlay that moves the kernel's chip select off GPIO7; with an
// older overlay GPIO7 belongs to the kernel and only the CE0 wait is left.
//
// The Pi's SPI block also drives SCLK (and MOSI) whenever the pins are in ALT0,
// between transfers too, against the FPGA during the Amiga's flash reads. So
// GPIO10/11 stay inputs (the overlay pulls SCLK down, MOSI up) and are switched
// to ALT0 only while this Pi has the bus.
#define CE0_WAIT_FIRST_US   10
#define CE0_WAIT_MAX_US     1000        // the Amiga reads the flash in windows of a few ms
#define CE0_WAIT_WARN_US    (1000*1000)
#define TF_PIN_SEL          7
#define TF_SEL_SETTLE_NS    2000
static int tf_sel_fd = -1;          // our select (GPIO7), -1 = the kernel has it
static volatile uint32_t *tf_gpio = NULL;   // /dev/gpiomem: GPIO10/11 function
static int init_gpio_irq();

// True if CE0 is asserted, or its state can't be read.
static bool ce0_asserted()
{
    gpio_v2_line_values gpio_state = { 0, (uint64_t)-1 };
    if (ioctl(gpio_irq_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &gpio_state))
        return true;
    return gpio_state.bits == 0;
}

static int64_t elapsed_ns(const struct timespec *t0)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)(t.tv_sec - t0->tv_sec) * 1000000000LL + (t.tv_nsec - t0->tv_nsec);
}

static void tf_set_sel(bool on)
{
    gpio_v2_line_values v = { on ? 0ULL : 1ULL, 1 };   // active low
    ioctl(tf_sel_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
}

// True while the FPGA is configured (CDONE, GPIO23, high). While it is held in
// reset or being flashed nothing may touch the bus: fomu-flash bit-bangs these
// same pins (GPIO10/11 as outputs), and an ALT0/input switch would cut its clock.
static bool tf_fpga_done()
{
    return !tf_gpio || (tf_gpio[13] & (1u << 23));      // GPLEV0
}

// GPFSEL1 bits 2:0 = GPIO10 (MOSI), bits 5:3 = GPIO11 (SCLK); ALT0 = 4
static void tf_spi_pins(bool spi)
{
    if (!tf_gpio)
        return;
    uint32_t v = tf_gpio[1] & ~0x3fu;
    if (spi)
        v |= (4u << 0) | (4u << 3);
    tf_gpio[1] = v;
}

static void init_tf_bus()
{
    // CE0 is read through the GPIO8 line, so request it before the first transfer
    init_gpio_irq();

    int chip_fd = open("/dev/gpiochip0", O_RDWR | O_CLOEXEC);
    if (chip_fd >= 0)
    {
        struct gpio_v2_line_request req = {};
        req.offsets[0] = TF_PIN_SEL;
        strcpy(req.consumer, "a314d-sel");
        req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        req.config.num_attrs = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        req.config.attrs[0].attr.values = 1;    // idle (high)
        req.config.attrs[0].mask = 1;
        req.num_lines = 1;
        if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) == 0)
            tf_sel_fd = req.fd;
        close(chip_fd);
    }
    if (tf_sel_fd < 0)
        logger_warning("GPIO7 belongs to the kernel (old spi-a314 overlay?): no SPI bus handshake with the TF4060, only a CE0 check\n");
    else
        logger_info("SPI bus handshake on GPIO7 / CE0\n");

    // the GPIO function registers are the BCM2835 layout up to the Pi 4 only
    char compat[256] = {0};
    int cfd = open("/proc/device-tree/compatible", O_RDONLY);
    if (cfd >= 0)
    {
        ssize_t n = read(cfd, compat, sizeof(compat) - 1);
        close(cfd);
        for (ssize_t i = 0; i < n; i++)
            if (!compat[i])
                compat[i] = ' ';
    }
    if (strstr(compat, "bcm2712"))
        logger_warning("Pi 5: SCLK/MOSI stay in ALT0 between transfers\n");
    else
    {
        int fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
        if (fd >= 0)
        {
            void *m = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            close(fd);
            if (m != MAP_FAILED)
                tf_gpio = (volatile uint32_t *)m;
        }
        if (!tf_gpio)
            logger_warning("Unable to map /dev/gpiomem: SCLK/MOSI stay in ALT0 between transfers\n");
    }
    if (tf_fpga_done())
        tf_spi_pins(false);
}

static void shutdown_tf_bus()
{
    tf_spi_pins(false);
    if (tf_sel_fd >= 0)
        tf_set_sel(false);
}

// Waits until this Pi may use the SPI bus: the FPGA configured and the Amiga
// off it. The callers can't handle a failed transfer, so there is no time
// limit; false only when SIGTERM is pending (a314d is being stopped).
static bool tf_bus_acquire()
{
    if (gpio_irq_fd < 0)
        return true;

    unsigned long waited = 0;
    __useconds_t timeout = CE0_WAIT_FIRST_US;
    bool warned = false;
    for (;;)
    {
        if (tf_fpga_done() && !ce0_asserted())
        {
            if (tf_sel_fd < 0)
                break;
            tf_set_sel(true);
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            while (elapsed_ns(&ts) < TF_SEL_SETTLE_NS)
                ;
            if (!ce0_asserted())
                break;
            tf_set_sel(false);      // the Amiga got the bus in the same instant
        }
        if (!warned && waited >= CE0_WAIT_WARN_US)
        {
            logger_warning(tf_fpga_done() ? "CE0 still asserted -- waiting to make an SPI transfer\n"
                                          : "The FPGA is not configured (CDONE low) -- waiting to make an SPI transfer\n");
            warned = true;
        }
        sigset_t pending;
        if (sigpending(&pending) == 0 && sigismember(&pending, SIGTERM))
            return false;
        usleep(timeout);
        waited += timeout;
        if (timeout < CE0_WAIT_MAX_US)
            timeout *= 2;
    }
    if (warned)
        logger_warning("CE0 is deasserted; continuing..\n");
    return true;
}
#endif

static int spi_transfer(int len)
{
#if defined(MODEL_TF)
    if (!tf_bus_acquire())
    {
        spi_error = true;
        return -1;
    }
    tf_spi_pins(true);
#endif
    struct spi_ioc_transfer tr =
    {
        .tx_buf = (uintptr_t)tx_buf,
        .rx_buf = (uintptr_t)rx_buf,
        .len = (uint32_t)len,
        .speed_hz = speed,
        .delay_usecs = 0,
        .bits_per_word = bits,
        .cs_change = 0,
    };

    int r = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
#if defined(MODEL_TF)
    tf_spi_pins(false);
    if (tf_sel_fd >= 0)
        tf_set_sel(false);
#endif
    if (r < 0)
        spi_error = true;
    return r;
}

static int spi_protocol_version()
{
    tx_buf[0] = (uint8_t)SPI_PROTO_VER_CMD;
    tx_buf[1] = 0;
    tx_buf[2] = 0;
    spi_transfer(1 /* 1 byte spi proto ver*/ + SPI_PROTO_HDR_LEN);
    logger_info("SPI protocol version = %d\n", rx_buf[SPI_PROTO_HDR_LEN]);
    return (int)rx_buf[SPI_PROTO_HDR_LEN];
}

#if defined(MODEL_TF)
// Longest data part of one shared RAM transfer. The Amiga's flash access waits
// while this Pi has the SPI bus (an access that does not poll for the grant is
// held on the bus), so long reads and writes go in pieces of ~180 us at 12 MHz.
#define TF_SPI_CHUNK 256
static void spi_read_shm_rxbuf_one(unsigned int address, unsigned int length);
static void spi_write_shm_one(unsigned int address, uint8_t *buf, unsigned int length);

static void spi_read_shm_rxbuf(unsigned int address, unsigned int length)
{
    if (length <= TF_SPI_CHUNK)
    {
        spi_read_shm_rxbuf_one(address, length);
        return;
    }
    static uint8_t data[sizeof(rx_buf)];
    for (unsigned int off = 0; off < length; off += TF_SPI_CHUNK)
    {
        unsigned int n = std::min(length - off, (unsigned int)TF_SPI_CHUNK);
        spi_read_shm_rxbuf_one(address + off, n);
        if (spi_error)
            return;
        memcpy(&data[off], &rx_buf[READ_SRAM_HDR_LEN], n);
    }
    memcpy(&rx_buf[READ_SRAM_HDR_LEN], data, length);
}

static void spi_write_shm(unsigned int address, uint8_t *buf, unsigned int length)
{
    for (unsigned int off = 0; off < length; off += TF_SPI_CHUNK)
    {
        spi_write_shm_one(address + off, buf + off, std::min(length - off, (unsigned int)TF_SPI_CHUNK));
        if (spi_error)
            return;
    }
}
#define spi_read_shm_rxbuf spi_read_shm_rxbuf_one
#define spi_write_shm spi_write_shm_one
#endif

static void spi_read_shm_rxbuf(unsigned int address, unsigned int length)
{
    logger_trace("SPI read mem address = %d length = %d\n", address, length);

    unsigned int header;
    if (spi_proto_ver >= 1)
        header = (READ_SRAM_CMD << 21) | (address & 0x1fffff);
    else
        header = (READ_SRAM_CMD << 20) | (address & 0xfffff);

    tx_buf[0] = (uint8_t)((header >> 16) & 0xff);
    tx_buf[1] = (uint8_t)((header >> 8) & 0xff);
    tx_buf[2] = (uint8_t)(header & 0xff);
	tx_buf[3] = 0;
	tx_buf[4] = 0;

    spi_transfer(length + READ_SRAM_HDR_LEN);
}

#if defined(MODEL_TF)
#undef spi_read_shm_rxbuf
#endif

static void spi_read_shm(unsigned char *data, unsigned int address, unsigned int length)
{
    spi_read_shm_rxbuf(address, length);
    memcpy(data, &rx_buf[READ_SRAM_HDR_LEN], length);
}

static void spi_write_shm(unsigned int address, uint8_t *buf, unsigned int length)
{
    logger_trace("SPI write mem address = %d length = %d\n", address, length);

    unsigned int header;
    if (spi_proto_ver >= 1)
        header = (WRITE_SRAM_CMD << 21) | (address & 0x1fffff);
    else
        header = (WRITE_SRAM_CMD << 20) | (address & 0xfffff);

    tx_buf[0] = (uint8_t)((header >> 16) & 0xff);
    tx_buf[1] = (uint8_t)((header >> 8) & 0xff);
    tx_buf[2] = (uint8_t)(header & 0xff);

    memcpy(&tx_buf[3], buf, length);
    spi_transfer(length + 3);
}
#if defined(MODEL_TF)
#undef spi_write_shm
#endif

#if defined(MODEL_TF)

static uint8_t spi_read_sint()
{
    tx_buf[0] = (uint8_t) (READ_SINT_CMD << 5);
    spi_transfer(1 /* 1 byte cmd */ + READ_SINT_HDR_LEN);
    return rx_buf[READ_SINT_HDR_LEN];
}

static void spi_write_sint(unsigned int data)
{
    data &= 0xf0;       // only upper bits used
    data >>= 4;         // only upper bits sent
    tx_buf[0] = (uint8_t) (WRITE_SINT_CMD << 5) | (data & 0xf);
    spi_transfer(1);
}

#else

static uint8_t spi_read_cmem(unsigned int address)
{
    if (spi_proto_ver == 1)
        tx_buf[0] = (uint8_t)((READ_CMEM_CMD << 5) | (address & 0xf));
    else
        tx_buf[0] = (uint8_t)((READ_CMEM_CMD << 4) | (address & 0xf));
    tx_buf[1] = 0;
    spi_transfer(2);
    logger_trace("SPI read cmem, address = %d, returned = %d\n", address, rx_buf[1]);
    return rx_buf[1];
}

static void spi_write_cmem(unsigned int address, unsigned int data)
{
    logger_trace("SPI write cmem, address = %d, data = %d\n", address, data);

    if (spi_proto_ver == 1)
        tx_buf[0] = (uint8_t)((WRITE_CMEM_CMD << 5) | (address & 0xf));
    else
        tx_buf[0] = (uint8_t)((WRITE_CMEM_CMD << 4) | (address & 0xf));
    tx_buf[1] = (uint8_t)(data & 0xf);
    spi_transfer(2);
}

static uint8_t spi_ack_irq()
{
    logger_trace("SPI ack_irq\n");
    return spi_read_cmem(R_EVENTS_ADDRESS);
}

static void spi_read_base_address()
{
    have_base_address = false;

    unsigned int ba1 = 0;
    for (int i = 0; i < BASE_ADDRESS_LEN; i++)
        ba1 |= spi_read_cmem(i) << (i * 4);

    if ((ba1 & 1) == 1)
    {
        unsigned int ba2 = 0;
        for (int i = 0; i < BASE_ADDRESS_LEN; i++)
            ba2 |= spi_read_cmem(i) << (i * 4);

        if (ba1 == ba2)
        {
            have_base_address = true;
            base_address = ba1 & ~1;
        }
    }
}
#endif

#define read_shm spi_read_shm
#define write_shm spi_write_shm
#endif

#if defined(MODEL_FE) || defined(MODEL_CP)
static int create_dev_gpiomem_mapping()
{
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        logger_error("Unable to open /dev/gpiomem\n");
        return -1;
    }

    void *gpio_map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    close(fd);

    if (gpio_map == MAP_FAILED)
    {
        logger_error("mmap failed, errno = %d\n", errno);
        return -1;
    }

    gpio = (volatile unsigned int *)gpio_map;
    return 0;
}

static void set_gpio_pull_mode(int mask, int mode)
{
    *(gpio + 37) = mode;
    usleep(50);
    *(gpio + 38) = mask;
    usleep(50);
    *(gpio + 38) = 0;
    *(gpio + 37) = 0;
    usleep(50);
}
#endif

#if defined(MODEL_FE)
static inline void set_dir_read()
{
    if (current_dir == 1)
    {
        *(gpio + 0) = (*(gpio + 0) & ~GPIO_DIR_0_MASK) | GPIO_DIR_0_DIN;
        *(gpio + 1) = GPIO_DIR_1_DIN;
        current_dir = 0;
    }
}

static inline void set_dir_write()
{
    if (current_dir == 0)
    {
        *(gpio + 0) = (*(gpio + 0) & ~GPIO_DIR_0_MASK) | GPIO_DIR_0_DOUT;
        *(gpio + 1) = GPIO_DIR_1_DOUT;
        current_dir = 1;
    }
}

static unsigned int read_sram_reg(unsigned int reg)
{
    set_dir_read();

    *(gpio + 7) = (reg << PIN_A(0)) | (1 << PIN_REQ);

    // Delay so that the ACK pin is sure to be negated after previous access.
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;

    // Additional delay to hopefully do only one read when there is no contention.
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;

    unsigned int value;
    while (!((value = *(gpio + 13)) & (1 << PIN_ACK)))
        ;

    value = (value >> PIN_D(0)) & 0xffff;

    *(gpio + 10) = (1 << PIN_REQ);
    *(gpio + 10) = (15 << PIN_A(0)) | (1 << PIN_WR) | (0xffff << PIN_D(0));

    return value;
}

static void write_sram_reg(unsigned int reg, unsigned int value)
{
    set_dir_write();

    *(gpio + 7) = (reg << PIN_A(0)) | (1 << PIN_REQ) | (1 << PIN_WR) | ((value & 0xffff) << PIN_D(0));

    // Delay so that the ACK pin is sure to be negated after previous access.
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;

    // Additional delay to hopefully do only one read when there is no contention.
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;

    while (!(*(gpio + 13) & (1 << PIN_ACK)))
        ;

    *(gpio + 10) = (1 << PIN_REQ);
    *(gpio + 10) = (15 << PIN_A(0)) | (1 << PIN_WR) | (0xffff << PIN_D(0));
}

static unsigned int read_reg(unsigned int reg)
{
    set_dir_read();

    *(gpio + 7) = (reg << PIN_A(0)) | (1 << PIN_REQ);
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;
    *(gpio + 7) = 0;

    unsigned int value = *(gpio + 13);
    value = (value >> PIN_D(0)) & 0xffff;

    *(gpio + 10) = (1 << PIN_REQ);
    *(gpio + 7) = 0;
    *(gpio + 10) = (15 << PIN_A(0)) | (1 << PIN_WR) | (0xffff << PIN_D(0));

    return value;
}

static void write_reg(unsigned int reg, unsigned int value)
{
    set_dir_write();

    *(gpio + 7) = (reg << PIN_A(0)) | (1 << PIN_REQ) | (1 << PIN_WR) | ((value & 0xffff) << PIN_D(0));
    *(gpio + 7) = 0;
    *(gpio + 10) = (1 << PIN_REQ);
    *(gpio + 7) = 0;
    *(gpio + 10) = (15 << PIN_A(0)) | (1 << PIN_WR) | (0xffff << PIN_D(0));
}

static void set_address(unsigned int address)
{
    if ((address & 0xffff) != (current_address & 0xffff))
        write_reg(REG_ADDR_LO, address & 0xffff);

    if (((address >> 16) & 0x3) != ((current_address >> 16) & 0x3))
        write_reg(REG_ADDR_HI, (address >> 16) & 0x3);

    current_address = address;
}

static void gpio_read_shm(unsigned char *data, unsigned int address, unsigned int length)
{
    set_address(address);

    current_address = (address + length) & 0x3ffff;

    unsigned int v;

    if (address & 1)
    {
        v = read_sram_reg(REG_SRAM_BYTE);
        *data++ = v & 0xff;
        length--;
    }

    while (length >= 2)
    {
        v = read_sram_reg(REG_SRAM_WORD);
        *data++ = (v >> 8) & 0xff;
        *data++ = v & 0xff;
        length -= 2;
    }

    if (length)
    {
        v = read_sram_reg(REG_SRAM_BYTE);
        *data++ = (v >> 8) & 0xff;
    }
}

static void gpio_write_shm(unsigned int address, unsigned char *data, unsigned int length)
{
    set_address(address);

    current_address = (address + length) & 0x3ffff;

    unsigned int v;

    if (address & 1)
    {
        v = *data++;
        write_sram_reg(REG_SRAM_BYTE, v);
        length--;
    }

    while (length >= 2)
    {
        v = ((*data++) << 8) | (*data++);
        write_sram_reg(REG_SRAM_WORD, v);
        length -= 2;
    }

    if (length)
    {
        v = (*data++) << 8;
        write_sram_reg(REG_SRAM_BYTE, v);
    }
}

#define write_shm gpio_write_shm
#define read_shm gpio_read_shm

static int init_gpio()
{
    if (create_dev_gpiomem_mapping())
        return -1;

    // Set pin directions.
    *(gpio + 0) = (*(gpio + 0) & ~GPIO_DIR_0_MASK) | GPIO_DIR_0_DIN;
    *(gpio + 1) = (*(gpio + 1) & ~GPIO_DIR_1_MASK) | GPIO_DIR_1_DIN;
    *(gpio + 2) = (*(gpio + 2) & ~GPIO_DIR_2_MASK) | GPIO_DIR_2;

    *(gpio + 10) = 0x0ffffff0;

    set_gpio_pull_mode((1 << PIN_ACK) | (1 << PIN_IRQ), GPIO_PULL_DOWN);
    set_gpio_pull_mode((15 << PIN_A(0)) | (1 << PIN_REQ) | (1 << PIN_WR) | (0xffff << PIN_D(0)), GPIO_PULL_NONE);

    current_dir = 0;

    // Set address pointer.
    write_reg(REG_ADDR_LO, 0);
    write_reg(REG_ADDR_HI, 0);

    current_address = 0;

    return 0;
}

static void shutdown_gpio()
{
    // Let Linux take care of this.
}

#elif defined(MODEL_CP)
static void gpio_write_reg(unsigned int reg, unsigned int value)
{
    if (current_dir == 0)
    {
        *(gpio + 7) = (1 << PIN_WR);
        *(gpio + 1) = GPIO_DIR_1_DOUT;
        *(gpio + 2) = GPIO_DIR_2_DOUT;
        current_dir = 1;
    }

    while ((*(gpio + 13) & (1 << PIN_ACK)))
        ;

    *(gpio + 7) = ((value & 0xff) << PIN_D(0)) | (reg << PIN_A(0)) | (1 << PIN_REQ);

    while (!(*(gpio + 13) & (1 << PIN_ACK)))
        ;

    *(gpio + 10) = PINS_OUT_NOT_WR;
}

static unsigned int gpio_read_reg(unsigned int reg)
{
    if (current_dir == 1)
    {
        *(gpio + 1) = GPIO_DIR_1_DIN;
        *(gpio + 2) = GPIO_DIR_2_DIN;
        *(gpio + 10) = (1 << PIN_WR);
        current_dir = 0;
    }

    while ((*(gpio + 13) & (1 << PIN_ACK)))
        ;

    *(gpio + 7) = (reg << PIN_A(0)) | (1 << PIN_REQ);

    unsigned int value;
    while (!((value = *(gpio + 13)) & (1 << PIN_ACK)))
        ;

    value = (value >> PIN_D(0)) & 0xff;

    *(gpio + 10) = PINS_OUT_NOT_WR;

    return value;
}

static void clear_pi_irq()
{
    gpio_write_reg(REG_IRQ, REG_IRQ_CLR | REG_IRQ_PI);
}

static void set_cp_irq()
{
    gpio_write_reg(REG_IRQ, REG_IRQ_SET | REG_IRQ_CP);
}

static void clear_cp_irq()
{
    gpio_write_reg(REG_IRQ, REG_IRQ_CLR | REG_IRQ_CP);
}

static void gpio_set_address(unsigned short address)
{
    if ((address & 0xff) != (current_address & 0xff))
        gpio_write_reg(REG_ADDR_LO, address & 0xff);

    if (((address >> 8) & 0xff) != ((current_address >> 8) & 0xff))
        gpio_write_reg(REG_ADDR_HI, (address >> 8) & 0xff);

    current_address = address;
}

static void gpio_write_shm(unsigned short address, unsigned char *data, unsigned short length)
{
    gpio_set_address(address);

    for (int i = 0; i < length; i++)
        gpio_write_reg(REG_SRAM, *data++);

    current_address += length;
}

static void gpio_read_shm(unsigned char *data, unsigned short address, unsigned short length)
{
    gpio_set_address(address);

    for (int i = 0; i < length; i++)
        *data++ = gpio_read_reg(REG_SRAM);

    current_address += length;
}

#define write_shm gpio_write_shm
#define read_shm gpio_read_shm

static int init_gpio()
{
    if (create_dev_gpiomem_mapping())
        return -1;

    // Set pin directions.
    // Inputs: PIN_IRQ, PIN_ACK, PIN_D(x), unused pins.
    // Outputs: PIN_REQ, PIN_A(x), PIN_WR, pin 29 (connects to LED on Pi3).
    // Alt0: PIN_CLK.
    // Directions of pins 0, 1, 3, 5, 6, 7 are left unchanged.
    *(gpio + 0) = (*(gpio + 0) & ~GPIO_DIR_0_MASK) | GPIO_DIR_0;
    *(gpio + 1) = GPIO_DIR_1_DIN;
    *(gpio + 2) = GPIO_DIR_2_DIN;

    set_gpio_pull_mode((1 << PIN_ACK) | (1 << PIN_IRQ), GPIO_PULL_DOWN);
    set_gpio_pull_mode(PINS_OUT | (1 << PIN_CLK), GPIO_PULL_NONE);

    current_dir = 0;

    // Set address pointer.
    gpio_write_reg(REG_ADDR_LO, 0);
    gpio_write_reg(REG_ADDR_HI, 0);

    current_address = 0;

    return 0;
}

static void shutdown_gpio()
{
    // Let Linux take care of this.
}
#endif

static int init_gpio_irq()
{
    if (gpio_irq_fd >= 0)
        return 0;       // TF: already requested by init_tf_bus()

    int chip_fd = open("/dev/gpiochip0", O_RDWR);
    if (chip_fd < 0)
        return -1;

    struct gpio_v2_line_request line_req = {
        .offsets = {IRQ_GPIO},
        .consumer = {'a', '3', '1', '4', 'd', 0},
        .config = {
            .flags = GPIO_V2_LINE_FLAG_INPUT | IRQ_GPIO_EDGE,
        },
        .num_lines = 1,
    };

    int err = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &line_req);
    if (err == -1)
        return -2;

    gpio_irq_fd = line_req.fd;
    if (gpio_irq_fd < 0)
        return -3;

    return 0;
}

static void shutdown_gpio_irq()
{
    // No-op, Linux releases resources.
}

static int init_server_socket()
{
    server_socket = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (server_socket == -1)
    {
        logger_error("Failed to create server socket\n");
        return -1;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(7110);

    int res = bind(server_socket, (struct sockaddr *)&address, sizeof(address));
    if (res < 0)
    {
        logger_error("Bind to localhost:7110 failed\n");
        return -1;
    }

    listen(server_socket, 16);

    return 0;
}

static void shutdown_server_socket()
{
    if (server_socket != -1)
        close(server_socket);
    server_socket = -1;
}

static void sigterm_handler(int signo)
{
}

static void init_sigterm()
{
    sigset_t ss;
    sigemptyset(&ss);
    sigaddset(&ss, SIGTERM);
    sigprocmask(SIG_BLOCK, &ss, &original_sigset);

    struct sigaction sa;
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGTERM, &sa, NULL);
}

static int init_driver()
{
    init_sigterm();

    if (init_server_socket() != 0)
        return -1;

#if defined(MODEL_TD) || defined(MODEL_TF)
    if (init_spi() != 0)
        return -1;

    if (check_spidev_bufsiz())
        logger_warning("The spidev.bufsiz argument in /boot/cmdline.txt is set incorrectly, it should be 65536\n");

#if defined(MODEL_TF)
    const uint32_t limit = 12*1000*1000;
    if (speed > limit)
    {
        speed = limit;
        logger_info("SPI speed limited to %d\n", speed);
    }
    init_tf_bus();
#endif

    spi_proto_ver = spi_protocol_version();
#elif defined(MODEL_FE) || defined(MODEL_CP)
    if (init_gpio() != 0)
        return -1;
#endif

    if (init_gpio_irq() != 0)
        return -1;

    epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd == -1)
        return -1;

    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = gpio_irq_fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, gpio_irq_fd, &ev) != 0)
        return -1;

    ev.events = EPOLLIN;
    ev.data.fd = server_socket;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, server_socket, &ev) != 0)
        return -1;

    return 0;
}

static void shutdown_driver()
{
    if (epfd != -1)
        close(epfd);

    shutdown_gpio_irq();
#if defined(MODEL_TD)
    shutdown_spi();
#elif defined(MODEL_TF)
    shutdown_tf_bus();      // SCLK/MOSI parked, GPIO7 idle
#elif defined(MODEL_FE) || defined(MODEL_CP)
    shutdown_gpio();
#endif
    shutdown_server_socket();
}

void create_and_send_msg(ClientConnection *cc, int type, int stream_id, uint8_t *data, int length)
{
    MessageBuffer mb;
    mb.pos = 0;
    mb.data.resize(sizeof(MessageHeader) + length);

    MessageHeader *mh = (MessageHeader *)&mb.data[0];
    mh->length = length;
    mh->stream_id = stream_id;
    mh->type = type;
    if (length && data)
        memcpy(&mb.data[sizeof(MessageHeader)], data, length);

    if (!cc->message_queue.empty())
    {
        cc->message_queue.push_back(std::move(mb));
        return;
    }

    while (1)
    {
        int left = mb.data.size() - mb.pos;
        uint8_t *src = &mb.data[mb.pos];
        ssize_t r = write(cc->fd, src, left);
        if (r == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                cc->message_queue.push_back(std::move(mb));
                return;
            }
            else if (errno == ECONNRESET)
            {
                // Do not close connection here; it will get done at some other place.
                return;
            }
            else
            {
                logger_error("Write failed unexpectedly with errno = %d\n", errno);
                exit(-1);
            }
        }

        mb.pos += r;
        if (r == left)
        {
            return;
        }
    }
}

static void handle_msg_register_req(ClientConnection *cc)
{
    uint8_t result = MSG_FAIL;

    std::string service_name((char *)&cc->payload[0], cc->payload.size());

    auto it = services.begin();
    for (; it != services.end(); it++)
        if (it->name == service_name)
            break;

    if (it == services.end())
    {
        services.emplace_back();

        RegisteredService &srv = services.back();
        srv.cc = cc;
        srv.name = std::move(service_name);

        result = MSG_SUCCESS;
    }

    create_and_send_msg(cc, MSG_REGISTER_RES, 0, &result, 1);
}

static void handle_msg_deregister_req(ClientConnection *cc)
{
    uint8_t result = MSG_FAIL;

    std::string service_name((char *)&cc->payload[0], cc->payload.size());

    for (auto it = services.begin(); it != services.end(); it++)
    {
        if (it->name == service_name && it->cc == cc)
        {
            services.erase(it);
            result = MSG_SUCCESS;
            break;
        }
    }

    create_and_send_msg(cc, MSG_DEREGISTER_RES, 0, &result, 1);
}

static void handle_msg_read_mem_req(ClientConnection *cc)
{
    if (cc->payload.size() != 8 || *(uint32_t *)&(cc->payload[4]) > sizeof(rx_buf) - 16)
    {
        logger_error("Malformed memory read request -- closing the client connection\n");
        failed_cc = cc;
        return;
    }

    uint32_t address = *(uint32_t *)&(cc->payload[0]);
    uint32_t length = *(uint32_t *)&(cc->payload[4]);

    if (length == 0)
    {
        // Nothing to transfer (the FE backend can't do a zero-length one).
        create_and_send_msg(cc, MSG_READ_MEM_RES, 0, nullptr, 0);
        return;
    }

    spi_error = false;
#if defined(MODEL_TD) || defined(MODEL_TF)
    // The data stays in rx_buf, after the header (no memcpy, which on TF
    // would also overlap).
    spi_read_shm_rxbuf(address, length);
    uint8_t *data = &rx_buf[READ_SRAM_HDR_LEN];
#else
    read_shm(rx_buf, address, length);
    uint8_t *data = rx_buf;
#endif
    if (spi_error)
    {
        logger_error("SPI transfer failed for a memory read -- closing the client connection\n");
        failed_cc = cc;
        return;
    }

    create_and_send_msg(cc, MSG_READ_MEM_RES, 0, data, length);
}

static void handle_msg_write_mem_req(ClientConnection *cc)
{
#if defined(MODEL_TD) || defined(MODEL_TF)
    // SPI writes go through tx_buf; FE and CP write straight from the payload.
    bool too_long = cc->payload.size() - 4 > sizeof(tx_buf) - 16;
#else
    bool too_long = false;
#endif
    if (cc->payload.size() < 4 || too_long)
    {
        logger_error("Malformed memory write request -- closing the client connection\n");
        failed_cc = cc;
        return;
    }

    uint32_t address = *(uint32_t *)&(cc->payload[0]);
    uint32_t length = cc->payload.size() - 4;

    if (length == 0)
    {
        create_and_send_msg(cc, MSG_WRITE_MEM_RES, 0, nullptr, 0);
        return;
    }

    spi_error = false;
    write_shm(address, &(cc->payload[4]), length);
    if (spi_error)
    {
        logger_error("SPI transfer failed for a memory write -- closing the client connection\n");
        failed_cc = cc;
        return;
    }

    create_and_send_msg(cc, MSG_WRITE_MEM_RES, 0, nullptr, 0);
}

static LogicalChannel *get_associated_channel_by_stream_id(ClientConnection *cc, int stream_id)
{
    for (auto ch : cc->associations)
    {
        if (ch->stream_id == stream_id)
            return ch;
    }
    return nullptr;
}

static void handle_msg_connect(ClientConnection *cc)
{
    // We currently don't handle that a client tries to connect to a service on the Amiga.
}

static void handle_msg_connect_response(ClientConnection *cc)
{
    LogicalChannel *ch = get_associated_channel_by_stream_id(cc, cc->header.stream_id);
    if (!ch)
        return;

    create_and_enqueue_packet(ch, PKT_CONNECT_RESPONSE, &cc->payload[0], cc->payload.size());

    if (cc->payload[0] != CONNECT_OK)
        remove_association(ch);
}

static void handle_msg_data(ClientConnection *cc)
{
    LogicalChannel *ch = get_associated_channel_by_stream_id(cc, cc->header.stream_id);
    if (!ch)
        return;

    create_and_enqueue_packet(ch, PKT_DATA, &cc->payload[0], cc->header.length);
}

static void handle_msg_eos(ClientConnection *cc)
{
    LogicalChannel *ch = get_associated_channel_by_stream_id(cc, cc->header.stream_id);
    if (!ch || ch->got_eos_from_client)
        return;

    ch->got_eos_from_client = true;

    create_and_enqueue_packet(ch, PKT_EOS, nullptr, 0);

    if (ch->got_eos_from_ami)
        remove_association(ch);
}

static void handle_msg_reset(ClientConnection *cc)
{
    LogicalChannel *ch = get_associated_channel_by_stream_id(cc, cc->header.stream_id);
    if (!ch)
        return;

    remove_association(ch);

    clear_packet_queue(ch);
    create_and_enqueue_packet(ch, PKT_RESET, nullptr, 0);
}

static void handle_received_message(ClientConnection *cc)
{
    switch (cc->header.type)
    {
    case MSG_REGISTER_REQ:
        handle_msg_register_req(cc);
        break;
    case MSG_DEREGISTER_REQ:
        handle_msg_deregister_req(cc);
        break;
    case MSG_READ_MEM_REQ:
        handle_msg_read_mem_req(cc);
        break;
    case MSG_WRITE_MEM_REQ:
        handle_msg_write_mem_req(cc);
        break;
    case MSG_CONNECT:
        handle_msg_connect(cc);
        break;
    case MSG_CONNECT_RESPONSE:
        handle_msg_connect_response(cc);
        break;
    case MSG_DATA:
        handle_msg_data(cc);
        break;
    case MSG_EOS:
        handle_msg_eos(cc);
        break;
    case MSG_RESET:
        handle_msg_reset(cc);
        break;
    default:
        // This is bad, probably should disconnect from client.
        logger_warning("Received a message of unknown type from client\n");
        break;
    }
}

static void close_and_remove_connection(ClientConnection *cc)
{
    shutdown(cc->fd, SHUT_WR);
    close(cc->fd);

    {
        auto it = services.begin();
        while (it != services.end())
        {
            if (it->cc == cc)
                it = services.erase(it);
            else
                it++;
        }
    }

    {
        auto it = cc->associations.begin();
        while (it != cc->associations.end())
        {
            auto ch = *it;

            clear_packet_queue(ch);
            create_and_enqueue_packet(ch, PKT_RESET, nullptr, 0);

            ch->association = nullptr;
            ch->stream_id = 0;

            it = cc->associations.erase(it);
        }
    }

    for (auto it = connections.begin(); it != connections.end(); it++)
    {
        if (&(*it) == cc)
        {
            connections.erase(it);
            break;
        }
    }
}

static void remove_association(LogicalChannel *ch)
{
    auto &ass = ch->association->associations;
    ass.erase(std::find(ass.begin(), ass.end(), ch));

    ch->association = nullptr;
    ch->stream_id = 0;
}

static void clear_packet_queue(LogicalChannel *ch)
{
    if (!ch->packet_queue.empty())
    {
        ch->packet_queue.clear();
        send_queue.erase(std::find(send_queue.begin(), send_queue.end(), ch));
    }
}

static void create_and_enqueue_packet(LogicalChannel *ch, uint8_t type, uint8_t *data, uint8_t length)
{
    if (ch->packet_queue.empty())
        send_queue.push_back(ch);

    ch->packet_queue.emplace_back();

    PacketBuffer &pb = ch->packet_queue.back();
    pb.type = type;
    pb.data.resize(length);
    if (data && length)
        memcpy(&pb.data[0], data, length);
}

static void handle_pkt_reset(int channel_id);
static void remove_channel_if_not_associated_and_empty_pq(int channel_id);

static void handle_pkt_connect(int channel_id, uint8_t *data, int plen)
{
    bool in_use = false;
    for (auto &ch : channels)
    {
        if (ch.channel_id == channel_id)
        {
            in_use = true;
            break;
        }
    }

    if (in_use)
    {
        // The Amiga reuses a stream id only after it has sent a RESET for it,
        // so this channel is left over from an Amiga reset that a314d did not
        // see (e.g. while backing off for CE0). Reset just this channel, as
        // for a PKT_RESET, and accept the CONNECT: other channels may already
        // be in use by the new boot.
        logger_warning("Received a CONNECT packet on a channel that was believed to be previously allocated -- missed Amiga reset? Resetting that channel\n");
        handle_pkt_reset(channel_id);
        remove_channel_if_not_associated_and_empty_pq(channel_id);
    }

    channels.emplace_back();

    auto &ch = channels.back();

    ch.channel_id = channel_id;
    ch.association = nullptr;
    ch.stream_id = 0;
    ch.got_eos_from_ami = false;
    ch.got_eos_from_client = false;

    std::string service_name((char *)data, plen);

    for (auto &srv : services)
    {
        if (srv.name == service_name)
        {
            ClientConnection *cc = srv.cc;

            ch.association = cc;
            ch.stream_id = cc->next_stream_id;

            cc->next_stream_id += 2;
            cc->associations.push_back(&ch);

            create_and_send_msg(ch.association, MSG_CONNECT, ch.stream_id, data, plen);
            return;
        }
    }

    for (auto &on_demand : on_demand_services)
    {
        if (on_demand.service_name == service_name)
        {
            int fds[2];
            int status = socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
            if (status != 0)
            {
                logger_error("Unexpectedly not able to create socket pair.\n");
                exit(-1);
            }

            pid_t child = fork();
            if (child == -1)
            {
                logger_error("Unexpectedly was not able to fork.\n");
                exit(-1);
            }
            else if (child == 0)
            {
                close(fds[0]);
                int fd = fds[1];

                std::vector<std::string> args(on_demand.arguments);
                args.push_back("-ondemand");
                args.push_back(std::to_string(fd));
                std::vector<const char *> args_arr;
                for (auto &arg : args)
                    args_arr.push_back(arg.c_str());
                args_arr.push_back(nullptr);

                execvp(on_demand.program.c_str(), (char* const*) &args_arr[0]);
            }
            else
            {
                close(fds[1]);
                int fd = fds[0];

                int status = fcntl(fd, F_SETFD, fcntl(fd, F_GETFD, 0) | FD_CLOEXEC);
                if (status == -1)
                {
                    logger_error("Unexpectedly unable to set close-on-exec flag on client socket descriptor; errno = %d\n", errno);
                    exit(-1);
                }

                status = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
                if (status == -1)
                {
                    logger_error("Unexpectedly unable to set client socket to non blocking; errno = %d\n", errno);
                    exit(-1);
                }

                int flag = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

                connections.emplace_back();

                ClientConnection &cc = connections.back();
                cc.fd = fd;
                cc.next_stream_id = 1;
                cc.bytes_read = 0;

                struct epoll_event ev;
                ev.events = EPOLLIN | EPOLLOUT | EPOLLET;
                ev.data.fd = fd;
                if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) != 0)
                {
                    logger_error("epoll_ctl() failed unexpectedly with errno = %d\n", errno);
                    exit(-1);
                }

                services.emplace_back();

                RegisteredService &srv = services.back();
                srv.cc = &cc;
                srv.name = std::move(service_name);

                ch.association = &cc;
                ch.stream_id = cc.next_stream_id;

                cc.next_stream_id += 2;
                cc.associations.push_back(&ch);

                create_and_send_msg(ch.association, MSG_CONNECT, ch.stream_id, data, plen);
                return;
            }
        }
    }

    logger_warning("service_name unknown; '%s'\n", service_name.c_str());

    uint8_t response = CONNECT_UNKNOWN_SERVICE;
    create_and_enqueue_packet(&ch, PKT_CONNECT_RESPONSE, &response, 1);
}

static void handle_pkt_data(int channel_id, uint8_t *data, int plen)
{
    for (auto &ch : channels)
    {
        if (ch.channel_id == channel_id)
        {
            if (ch.association != nullptr && !ch.got_eos_from_ami)
                create_and_send_msg(ch.association, MSG_DATA, ch.stream_id, data, plen);

            break;
        }
    }
}

static void handle_pkt_eos(int channel_id)
{
    for (auto &ch : channels)
    {
        if (ch.channel_id == channel_id)
        {
            if (ch.association != nullptr && !ch.got_eos_from_ami)
            {
                ch.got_eos_from_ami = true;

                create_and_send_msg(ch.association, MSG_EOS, ch.stream_id, nullptr, 0);

                if (ch.got_eos_from_client)
                    remove_association(&ch);
            }
            break;
        }
    }
}

static void handle_pkt_reset(int channel_id)
{
    for (auto &ch : channels)
    {
        if (ch.channel_id == channel_id)
        {
            clear_packet_queue(&ch);

            if (ch.association != nullptr)
            {
                create_and_send_msg(ch.association, MSG_RESET, ch.stream_id, nullptr, 0);
                remove_association(&ch);
            }

            break;
        }
    }
}

static void remove_channel_if_not_associated_and_empty_pq(int channel_id)
{
    for (auto it = channels.begin(); it != channels.end(); it++)
    {
        if (it->channel_id == channel_id)
        {
            if (it->association == nullptr && it->packet_queue.empty())
                channels.erase(it);

            break;
        }
    }
}

// DATA or EOS on a channel a314d does not know: the Amiga holds a stream from
// before an a314d restart. Answer with a RESET, so its client is told instead
// of waiting for ever. The channel goes away once the RESET is sent.
static void reset_unknown_channel(int channel_id, int ptype)
{
    for (auto &ch : channels)
    {
        if (ch.channel_id == channel_id)
            return;
    }

    logger_warning("Received a %s packet on unknown channel %d -- resetting it\n",
                   ptype == PKT_DATA ? "DATA" : "EOS", channel_id);

    channels.emplace_back();

    auto &ch = channels.back();

    ch.channel_id = channel_id;
    ch.association = nullptr;
    ch.stream_id = 0;
    ch.got_eos_from_ami = false;
    ch.got_eos_from_client = false;

    create_and_enqueue_packet(&ch, PKT_RESET, nullptr, 0);
}

// After a corrupt a2r: what was dropped may have been for any channel, so every
// channel is reset on both sides. Unlike close_all_logical_channels() (for an
// Amiga reset, when the Amiga's sockets are gone anyway), each one also gets a
// PKT_RESET, so an Amiga socket that only waits to receive is told too. A
// channel goes away once its RESET is sent.
static void reset_all_channels()
{
    for (auto &ch : channels)
    {
        clear_packet_queue(&ch);

        if (ch.association != nullptr)
        {
            create_and_send_msg(ch.association, MSG_RESET, ch.stream_id, nullptr, 0);
            remove_association(&ch);
        }

        create_and_enqueue_packet(&ch, PKT_RESET, nullptr, 0);
    }
}

static void handle_received_pkt(int ptype, int channel_id, uint8_t *data, int plen)
{
    if (ptype == PKT_DATA || ptype == PKT_EOS)
        reset_unknown_channel(channel_id, ptype);

    if (ptype == PKT_CONNECT)
        handle_pkt_connect(channel_id, data, plen);
    else if (ptype == PKT_DATA)
        handle_pkt_data(channel_id, data, plen);
    else if (ptype == PKT_EOS)
        handle_pkt_eos(channel_id);
    else if (ptype == PKT_RESET)
        handle_pkt_reset(channel_id);

    remove_channel_if_not_associated_and_empty_pq(channel_id);
}

static void receive_from_a2r()
{
    int head = channel_status[A2R_HEAD_OFFSET];
    int tail = channel_status[A2R_TAIL_OFFSET];
    int len = (tail - head) & 255;
    if (len == 0)
        return;

    spi_error = false;
    if (head < tail)
        read_shm(recv_buf, BASE_ADDRESS + A2R_BASE + head, len);
    else
    {
        read_shm(recv_buf, BASE_ADDRESS + A2R_BASE + head, 256 - head);

        if (tail != 0)
            read_shm(&recv_buf[len - tail], BASE_ADDRESS + A2R_BASE, tail);
    }
    if (spi_error)
    {
        // Not taken: the head stays, so the packets are read again next time.
        logger_error("SPI read of the a2r ring failed -- retrying later\n");
        return;
    }

    uint8_t *p = recv_buf;
    uint8_t *end = recv_buf + len;
    while (p < end)
    {
        // A header and payload that do not fit the bytes read are not a packet.
        if (end - p < 3 || end - p - 3 < p[0])
        {
            // What was dropped may have been for any stream, so none of them can
            // be trusted: reset them all, on both sides.
            logger_error("Malformed packet in a2r -- dropping it and resetting all channels\n");
            reset_all_channels();
            break;
        }
        uint8_t plen = *p++;
        uint8_t ptype = *p++;
        uint8_t channel_id = *p++;
        handle_received_pkt(ptype, channel_id, p, plen);
        p += plen;
    }

    channel_status[A2R_HEAD_OFFSET] = channel_status[A2R_TAIL_OFFSET];
    channel_status_updated |= A2R_HEAD_UPDATED;
}

static void flush_send_queue()
{
    int tail = channel_status[R2A_TAIL_OFFSET];
    int head = channel_status[R2A_HEAD_OFFSET];
    int len = (tail - head) & 255;
    int left = 255 - len;

    // The batch is built from a copy of the send order, without touching the
    // queues. The packets are taken off them only once the ring bytes and the
    // new r2a tail are in SRAM: after a failed SPI write they stay queued and
    // are sent again, instead of being lost or of a tail that shows stale bytes.
    std::list<LogicalChannel*> order(send_queue);
    std::map<LogicalChannel*, std::list<PacketBuffer>::iterator> next;
    int npackets = 0;
    int pos = 0;

    while (!order.empty())
    {
        LogicalChannel *ch = order.front();
        auto found = next.find(ch);
        auto pit = found == next.end() ? ch->packet_queue.begin() : found->second;
        PacketBuffer &pb = *pit;

        int ptype = pb.type;
        int plen = 3 + pb.data.size();

        if (left < plen)
            break;

        send_buf[pos++] = pb.data.size();
        send_buf[pos++] = ptype;
        send_buf[pos++] = ch->channel_id;
        memcpy(&send_buf[pos], &pb.data[0], pb.data.size());
        pos += pb.data.size();
        npackets++;

        order.pop_front();
        next[ch] = ++pit;
        if (pit != ch->packet_queue.end())
            order.push_back(ch);

        left -= plen;
    }

    int to_write = pos;
    if (!to_write)
        return;

    spi_error = false;

    uint8_t *p = send_buf;
    int at_end = 256 - tail;
    if (at_end < to_write)
    {
        write_shm(BASE_ADDRESS + R2A_BASE + tail, p, at_end);
        p += at_end;
        to_write -= at_end;
        tail = 0;
    }

    write_shm(BASE_ADDRESS + R2A_BASE + tail, p, to_write);
    tail = (tail + to_write) & 255;

    uint8_t new_tail = tail;
    if (!spi_error)
        write_shm(BASE_ADDRESS + CAP_BASE + R2A_TAIL_OFFSET, &new_tail, 1);

    if (spi_error)
    {
        logger_error("SPI write of the r2a ring failed -- packets kept, sent again later\n");
        return;
    }

    channel_status[R2A_TAIL_OFFSET] = tail;
    channel_status_updated |= R2A_TAIL_UPDATED;

    // Committed: take the packets off their queues, in the order they went.
    for (int i = 0; i < npackets; i++)
    {
        LogicalChannel *ch = send_queue.front();

        ch->packet_queue.pop_front();

        send_queue.pop_front();

        if (!ch->packet_queue.empty())
            send_queue.push_back(ch);
        else
            remove_channel_if_not_associated_and_empty_pq(ch->channel_id);
    }
}

// Sets spi_error if the read failed (the TF callers then stop).
static void read_channel_status()
{
    spi_error = false;
    read_shm(channel_status, BASE_ADDRESS + CAP_BASE, 4);
    channel_status_updated = 0;

    if (a2r_head_unwritten >= 0)
    {
        channel_status[A2R_HEAD_OFFSET] = a2r_head_unwritten;
        channel_status_updated |= A2R_HEAD_UPDATED;
    }
}

static void write_channel_status()
{
    if (channel_status_updated != 0)
    {
        // The r2a tail is already in SRAM (flush_send_queue() writes it).
        if ((channel_status_updated & A2R_HEAD_UPDATED) || a2r_head_unwritten >= 0)
        {
            spi_error = false;
            write_shm(BASE_ADDRESS + CAP_BASE + A2R_HEAD_OFFSET, &channel_status[A2R_HEAD_OFFSET], 1);
            if (spi_error)
            {
                spi_error = false;
                write_shm(BASE_ADDRESS + CAP_BASE + A2R_HEAD_OFFSET, &channel_status[A2R_HEAD_OFFSET], 1);
            }
            if (spi_error)
            {
                logger_error("SPI write of the a2r head failed -- retrying later\n");
                a2r_head_unwritten = channel_status[A2R_HEAD_OFFSET];
            }
            else
                a2r_head_unwritten = -1;
        }

#if defined(MODEL_TD)
        unsigned int events = 0;
        if (channel_status_updated & R2A_TAIL_UPDATED)
            events |= A_EVENT_R2A_TAIL;
        if (channel_status_updated & A2R_HEAD_UPDATED)
            events |= A_EVENT_A2R_HEAD;
        spi_write_cmem(A_EVENTS_ADDRESS, events);
#elif defined(MODEL_FE)
        uint32_t irq = 0;
        if (channel_status_updated & R2A_TAIL_UPDATED)
            irq |= IRQ_R2A_TAIL;
        if (channel_status_updated & A2R_HEAD_UPDATED)
            irq |= IRQ_A2R_HEAD;
        write_reg(REG_INT_REQ, IRQ_SET | irq);

        auto_clear_irq = true;
        auto_clear_irq_after = time(NULL) + 3;
#elif defined(MODEL_CP)
        set_cp_irq();

        auto_clear_irq = true;
        auto_clear_irq_after = time(NULL) + 3;
#elif defined(MODEL_TF)
        spi_write_sint(REG_IRQ_SET | REG_IRQ_AMIGA);
#endif
        channel_status_updated = 0;
    }
}

static void close_all_logical_channels()
{
    send_queue.clear();
    a2r_head_unwritten = -1;    // the ring it belonged to is gone

    auto it = channels.begin();
    while (it != channels.end())
    {
        LogicalChannel &ch = *it;

        if (ch.association != nullptr)
        {
            create_and_send_msg(ch.association, MSG_RESET, ch.stream_id, nullptr, 0);
            remove_association(&ch);
        }

        it = channels.erase(it);
    }
}

#if defined(MODEL_TD)
static void handle_a314_irq()
{
    uint8_t events = spi_ack_irq();
    if (events == 0)
        return;

    if ((events & R_EVENT_BASE_ADDRESS) || !have_base_address)
    {
        if (have_base_address && !channels.empty())
            logger_info("Base address was updated while logical channels are open -- closing channels\n");

        close_all_logical_channels();
        spi_read_base_address();
    }

    if (!have_base_address)
        return;

    read_channel_status();

    receive_from_a2r();
    flush_send_queue();

    write_channel_status();
}
#elif defined(MODEL_FE)
static void handle_a314_irq()
{
    write_reg(REG_INT_ENA, 0);

    uint32_t irq = read_reg(REG_INT_REQ) & (IRQ_BASE_ADDRESS | IRQ_R2A_HEAD | IRQ_A2R_TAIL);

    if (irq)
        write_reg(REG_INT_REQ, IRQ_CLR | irq);

    if (irq & IRQ_BASE_ADDRESS || !have_base_address)
    {
        base_address = read_reg(REG_CA_BASE_ADDR) << 2;
        have_base_address = base_address != 0x3fffc;

        if (!channels.empty())
            logger_info("Base address was updated while logical channels are open -- closing channels\n");

        close_all_logical_channels();
    }

    if (have_base_address)
    {
        read_channel_status();

        receive_from_a2r();
        flush_send_queue();

        write_channel_status();
    }

    write_reg(REG_INT_ENA, IRQ_BASE_ADDRESS | IRQ_R2A_HEAD | IRQ_A2R_TAIL);
}
#elif defined(MODEL_CP)
static void handle_a314_irq()
{
    // TODO: Currently there's a single notification event in either direction.
    // Could possibly have two events, one to signal that R2A_TAIL is updated
    // and one for A2R_HEAD.
    // Also in that case should add enable bits so that the other side can
    // mute notifications.

    clear_pi_irq();

    if (!read_a314_magic)
    {
        uint8_t values[2];
        read_shm(values, BASE_ADDRESS + CAP_BASE + 5, 2);
        if (values[0] != 0xa3 || values[1] != 0x14)
            return;

        read_shm(values, BASE_ADDRESS + CAP_BASE + 4, 1);
        restart_counter = values[0];

        read_a314_magic = true;
    }

    read_channel_status();

    uint8_t current_restart_counter;
    read_shm(&current_restart_counter, BASE_ADDRESS + CAP_BASE + 4, 1);

    if (current_restart_counter != restart_counter)
    {
        if (!channels.empty())
            logger_info("Restart counter was updated while logical channels are open -- closing channels\n");
        else
            logger_info("Restart counter was updated\n");

        close_all_logical_channels();
        restart_counter = current_restart_counter;

        read_channel_status();
    }

    receive_from_a2r();
    flush_send_queue();

    write_channel_status();
}
#elif defined(MODEL_TF)
static void handle_a314_irq()
{
    // check ICE40 bus activity, and back-off if CE0 is asserted
    {
        static bool old_ce0_state = 1;
        gpio_v2_line_values gpio_state = { 0, (uint64_t)-1 };
        if (ioctl(gpio_irq_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &gpio_state))
        {
            logger_warning("Unable to get GPIO state; %d / %s\n", errno, strerror(errno));
            return;
        }

        bool ce0_asserted = (gpio_state.bits == 0);

        if (old_ce0_state != ce0_asserted)
        {
            if (ce0_asserted)
                logger_warning("CE0 is asserted; backing off!\n");
            else
                logger_warning("CE0 is deasserted; continuing..\n");

            old_ce0_state = ce0_asserted;
        }

        static __useconds_t timeout = 10;
        if (ce0_asserted)
        {
            usleep(timeout);
            if (timeout < 100*1000)
                timeout *= 2;
            return;
        }
        else
        {
            timeout = 10;
        }
    }

    // if spi_proto_ver doesn't match most likely the ICE40 isn't running (amiga powered off?)
    if (spi_proto_ver != 2)
    {
        spi_proto_ver = spi_protocol_version();
        if (spi_proto_ver != 2)
        {
            usleep(500*1000);    // takes about 400-500ms to stabilize
            return;
        }
    }

    spi_error = false;
    uint8_t irq = spi_read_sint();
    if (spi_error)
        return;     // not a real value: try again on the next pass

    // RESET is cleared by the Amiga-side driver, and set again on Amiga reset
    if ((irq & REG_IRQ_RESET))
    {
        a2r_head_unwritten = -1;    // the Amiga sets up a new ring
        if ((irq & REG_IRQ_RPI))
        {
            logger_info("Amiga is probing the a314d side\n");
            spi_write_sint(REG_IRQ_RPI);
        }

        if (channels.empty())
        {
            usleep(10*1000);    // the amiga driver isn't active; sleep a bit..
            return;
        }

        logger_info("Amiga was reset while logical channels are open -- closing channels\n");
        close_all_logical_channels();
        spi_write_sint(REG_IRQ_RPI);    // reset any lingering irqs
        return;
    }

    {
        const uint64_t cooldown = 100*1000;
        static uint64_t sleep_counter = 0;

        bool idle_poll = false;
        if (!(irq & REG_IRQ_RPI))
        {
            // after N checks w/o activity, sleep some extra
            if (sleep_counter == cooldown)
                logger_info("PI became inactive!\n");

            if (sleep_counter > cooldown)
                usleep(1000);

            sleep_counter++;

            if (sleep_counter > 1000)
                usleep(1);                  // basic yield to not slam the poor PI

            // Fallback poll: look at the com area every 1000 checks (about
            // every 0.1 s just after activity, about once a second after the
            // cooldown) even without an interrupt, so a lost one can't leave
            // both sides waiting.
            if ((sleep_counter % 1000) != 0)
                return;
            idle_poll = true;
        }

        if (!idle_poll)
        {
            if (sleep_counter > cooldown)
                logger_info("PI became active!\n");

            sleep_counter = 0;
        }
    }

    spi_write_sint(REG_IRQ_RPI);

    read_channel_status();
    if (spi_error)
    {
        logger_error("SPI read of the channel status failed -- retrying later\n");
        return;
    }

    receive_from_a2r();
    flush_send_queue();

    write_channel_status();
}

// Before a client event publishes packets: the com area pointers kept from the
// last handle_a314_irq() may be from before an Amiga reset. Read them again,
// and publish nothing while the Amiga driver is not set up (RESET still set);
// if channels are open then, the Amiga was reset, so close them first.
// Nothing is published while the FPGA is busy either; handle_a314_irq() does
// it later.
static bool com_area_ready()
{
    // Not while the FPGA is busy (CE0) or not answering: handle_a314_irq()
    // backs off then, and publishes once it is back.
    gpio_v2_line_values gpio_state = { 0, (uint64_t)-1 };
    if (ioctl(gpio_irq_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &gpio_state) || gpio_state.bits == 0)
        return false;
    if (spi_proto_ver != 2)
        return false;

    spi_error = false;
    uint8_t irq = spi_read_sint();
    if (spi_error)
        return false;
    if (irq & REG_IRQ_RESET)
    {
        a2r_head_unwritten = -1;
        if (!channels.empty())
        {
            logger_info("Amiga was reset while logical channels are open -- closing channels\n");
            close_all_logical_channels();
        }
        return false;
    }

    read_channel_status();
    return !spi_error;
}
#endif

static void handle_client_connection_event(ClientConnection *cc, struct epoll_event *ev)
{
    if (ev->events & EPOLLERR)
    {
        logger_warning("Received EPOLLERR for client connection\n");
        close_and_remove_connection(cc);
        return;
    }

    if (ev->events & EPOLLIN)
    {
        while (1)
        {
            int left;
            uint8_t *dst;

            if (cc->payload.empty())
            {
                left = sizeof(MessageHeader) - cc->bytes_read;
                dst = (uint8_t *)&(cc->header) + cc->bytes_read;
            }
            else
            {
                left = cc->header.length - cc->bytes_read;
                dst = &cc->payload[cc->bytes_read];
            }

            ssize_t r = read(cc->fd, dst, left);
            if (r == -1)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;

                logger_error("Read failed unexpectedly with errno = %d\n", errno);
                exit(-1);
            }

            if (r == 0)
            {
                logger_info("Received End-of-File on client connection\n");
                close_and_remove_connection(cc);
                return;
            }
            else
            {
                cc->bytes_read += r;
                left -= r;
                if (!left)
                {
                    if (cc->payload.empty())
                    {
                        if (cc->header.length == 0)
                        {
                            logger_trace("header: length=%d, stream_id=%d, type=%d\n", cc->header.length, cc->header.stream_id, cc->header.type);
                            handle_received_message(cc);
                            if (failed_cc == cc)
                            {
                                failed_cc = nullptr;
                                close_and_remove_connection(cc);
                                return;
                            }
                        }
                        else
                        {
                            // Bounded before allocating: the largest real message is
                            // a memory write of a few hundred KB.
                            if (cc->header.length > MAX_CLIENT_MSG_LEN)
                            {
                                logger_error("Client message of %u bytes is too long -- closing the client connection\n", cc->header.length);
                                close_and_remove_connection(cc);
                                return;
                            }
                            cc->payload.resize(cc->header.length);
                        }
                    }
                    else
                    {
                        logger_trace("header: length=%d, stream_id=%d, type=%d\n", cc->header.length, cc->header.stream_id, cc->header.type);
                        handle_received_message(cc);
                        if (failed_cc == cc)
                        {
                            failed_cc = nullptr;
                            close_and_remove_connection(cc);
                            return;
                        }
                        cc->payload.clear();
                    }
                    cc->bytes_read = 0;
                }
            }
        }
    }

    if (ev->events & EPOLLOUT)
    {
        while (!cc->message_queue.empty())
        {
            MessageBuffer &mb = cc->message_queue.front();

            int left = mb.data.size() - mb.pos;
            uint8_t *src = &mb.data[mb.pos];
            ssize_t r = write(cc->fd, src, left);
            if (r == -1)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                else if (errno == ECONNRESET)
                {
                    close_and_remove_connection(cc);
                    return;
                }
                else
                {
                    logger_error("Write failed unexpectedly with errno = %d\n", errno);
                    exit(-1);
                }
            }

            mb.pos += r;
            if (r == left)
                cc->message_queue.pop_front();
        }
    }
}

static void handle_server_socket_ready()
{
    struct sockaddr_in address;
    int alen = sizeof(struct sockaddr_in);

    int fd = accept(server_socket, (struct sockaddr *)&address, (socklen_t *)&alen);
    if (fd < 0)
    {
        logger_error("Accept failed unexpectedly with errno = %d\n", errno);
        exit(-1);
    }

    int status = fcntl(fd, F_SETFD, fcntl(fd, F_GETFD, 0) | FD_CLOEXEC);
    if (status == -1)
    {
        logger_error("Unexpectedly unable to set close-on-exec flag on client socket descriptor; errno = %d\n", errno);
        exit(-1);
    }

    status = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    if (status == -1)
    {
        logger_error("Unexpectedly unable to set client socket to non blocking; errno = %d\n", errno);
        exit(-1);
    }

    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

    connections.emplace_back();

    ClientConnection &cc = connections.back();
    cc.fd = fd;
    cc.next_stream_id = 1;
    cc.bytes_read = 0;

    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLOUT | EPOLLET;
    ev.data.fd = fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) != 0)
    {
        logger_error("epoll_ctl() failed unexpectedly with errno = %d\n", errno);
        exit(-1);
    }
}

static void main_loop()
{
    handle_a314_irq();

    bool first_gpio_event = true;
    bool shutting_down = false;
    time_t force_shutdown_after;
    bool done = false;

#if defined(MODEL_FE) || defined(MODEL_CP)
    auto_clear_irq = true;
    auto_clear_irq_after = time(NULL) + 3;
#endif

    logger_trace("Entering main loop\n");

    while (!done)
    {
        int timeout = 1000;
#if defined(MODEL_TF)        
        sigset_t sigset;
        sigpending(&sigset);
        if (!sigismember(&sigset, SIGTERM))
            timeout = 0;
#endif
        struct epoll_event ev;
        int n = epoll_pwait(epfd, &ev, 1, timeout, &original_sigset);
        if (n == -1)
        {
            if (errno == EINTR)
            {
                logger_info("Received SIGTERM\n");

                shutdown_server_socket();

                while (!connections.empty())
                    close_and_remove_connection(&connections.front());

                flush_send_queue();
                write_channel_status();

                if (!shutting_down)
                    force_shutdown_after = time(NULL) + 10;

                shutting_down = true;
            }
            else
            {
                logger_error("epoll_pwait failed with unexpected errno = %d\n", errno);
                exit(-1);
            }
        }
        else if (n == 0)
        {
            // Timeout. Handle below.
#if defined(MODEL_TF)
            handle_a314_irq();
#endif
        }
        else
        {
            if (ev.data.fd == gpio_irq_fd)
            {
                struct gpio_v2_line_event ev;
                ssize_t rd = read(gpio_irq_fd, &ev, sizeof(ev));

                if (rd != sizeof(ev))
                {
                    logger_error("Read GPIO event, got %d bytes but expected %d\n", rd, sizeof(ev));
                    exit(-1);
                }

                logger_trace("GPIO interupted\n");
                handle_a314_irq();
            }
            else if (ev.data.fd == server_socket)
            {
                logger_trace("Epoll event: server socket is ready, events = %d\n", ev.events);
                handle_server_socket_ready();
            }
            else
            {
                logger_trace("Epoll event: client socket is ready, events = %d\n", ev.events);

                auto it = connections.begin();
                for (; it != connections.end(); it++)
                {
                    if (it->fd == ev.data.fd)
                        break;
                }

                if (it == connections.end())
                {
                    logger_error("Got notified about an event on a client connection that supposedly isn't currently open\n");
                    exit(-1);
                }

                ClientConnection *cc = &(*it);
                handle_client_connection_event(cc, &ev);

#if defined(MODEL_TF)
                // Most client events (e.g. memory requests) queue no packets:
                // then there is nothing to publish, so no SPI traffic.
                if ((!send_queue.empty() || a2r_head_unwritten >= 0) && com_area_ready())
#endif
                {
                    flush_send_queue();
                    write_channel_status();
                }
            }
        }

        time_t now = time(NULL);

        if (auto_clear_irq && now > auto_clear_irq_after)
        {
#if defined(MODEL_FE)
            write_reg(REG_INT_REQ, IRQ_CLR | IRQ_A2R_HEAD | IRQ_R2A_TAIL);
#elif defined(MODEL_CP)
            clear_cp_irq();
#endif
            auto_clear_irq = false;
        }

        if (shutting_down && (channels.empty() || now > force_shutdown_after))
            done = true;
    }
}

int main(int argc, char **argv)
{
    // Line-buffer stdout (info/warning) like stderr (errors), so the journal
    // keeps the real order and times of the log lines.
    setvbuf(stdout, NULL, _IOLBF, 0);

    std::string conf_filename("/etc/opt/a314/a314d.conf");

    int used_args = 1;

    if (argc >= 3 && strcmp(argv[1], "--loglevel") == 0)
    {
        used_args += 2;
        if (strcasecmp(argv[2], "trace") == 0) loglevel = LOGLEVEL_TRACE;
        else if (strcasecmp(argv[2], "debug") == 0) loglevel = LOGLEVEL_DEBUG;
        else if (strcasecmp(argv[2], "info") == 0) loglevel = LOGLEVEL_INFO;
        else if (strcasecmp(argv[2], "warning") == 0) loglevel = LOGLEVEL_WARNING;
        else if (strcasecmp(argv[2], "error") == 0) loglevel = LOGLEVEL_ERROR;
        else fprintf(stderr, "Invalid log level '%s', defaulting to INFO\n", argv[2]);
    }

    if (argc > used_args)
        conf_filename = argv[used_args];

    logger_info("a314d " A314D_VERSION "\n");

    load_config_file(conf_filename.c_str());

    if (init_driver() == 0)
        main_loop();
    shutdown_driver();
    return 0;
}
