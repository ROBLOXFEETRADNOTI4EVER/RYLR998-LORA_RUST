#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config, Uart};
use esp_println as _;
use static_cell::StaticCell;

extern crate alloc;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}


const IS_A_RECEIVER: bool = true;
const SEND_INTERVAL: Duration = Duration::from_millis(100);


pub struct LoraConfig {
    pub address:    u8,
    pub peer:       u8,
    pub band:       u32,
    pub network_id: u8,
    pub sf:         u8,
    pub bw:         u8,
    pub cr:         u8,
    pub preamble:   u8,
    pub power:      u8, // 0-22
}

impl LoraConfig {
    pub fn default_eu() -> Self {
        Self {
            address:    if IS_A_RECEIVER { 2 } else { 1 },
            peer:       if IS_A_RECEIVER { 1 } else { 2 },
            band:       868_000_000,
            network_id: 18,
            sf:         9,
            bw:         7,
            cr:         1,
            preamble:   7,
            power:      22,
        }
    }


    pub fn address(mut self, v: u8) -> Self    { self.address    = v; self }
    pub fn peer(mut self, v: u8) -> Self       { self.peer       = v; self }
    pub fn freq(mut self, v: u32) -> Self      { self.band       = v; self }
    pub fn network(mut self, v: u8) -> Self    { self.network_id = v; self }
    pub fn sf(mut self, v: u8) -> Self         { self.sf         = v; self }
    pub fn bw(mut self, v: u8) -> Self         { self.bw         = v; self }
    pub fn cr(mut self, v: u8) -> Self         { self.cr         = v; self }
    pub fn preamble(mut self, v: u8) -> Self   { self.preamble   = v; self }
    pub fn power(mut self, v: u8) -> Self      { self.power      = v; self }
}


#[derive(defmt::Format)]
pub struct LoraPacket {
    pub from: u8,
    pub data: heapless::String<64>,
    pub rssi: i16,
    pub snr:  i8,
}

pub struct Lora<U: Read + Write> {
    uart:   U,
    buf:    [u8; 256],
    config: LoraConfig,
}

impl<U: Read + Write> Lora<U> {
    pub fn new(uart: U, config: LoraConfig) -> Self {
        Self { uart, buf: [0u8; 256], config }
    }

    pub async fn at(&mut self, cmd: &[u8]) -> bool {
        let _ = embassy_time::with_timeout(
            Duration::from_millis(30),
            self.uart.read(&mut self.buf),
        ).await;

        self.uart.write_all(cmd).await.unwrap();

        let mut resp: heapless::String<128> = heapless::String::new();
        let deadline = Instant::now() + Duration::from_millis(1500);

        loop {
            if Instant::now() > deadline {
                info!("[AT] TIMEOUT: {=[u8]:a}", cmd);
                return false;
            }
            if let Ok(Ok(n)) = embassy_time::with_timeout(
                Duration::from_millis(200),
                self.uart.read(&mut self.buf),
            ).await {
                if n > 0 {
                    if let Ok(s) = core::str::from_utf8(&self.buf[..n]) {
                        let _ = resp.push_str(s);
                    }
                    if resp.contains("+OK")  { return true;  }
                    if resp.contains("+ERR") { return false; }
                }
            }
        }
    }

    pub async fn at_fmt(&mut self, args: core::fmt::Arguments<'_>) -> bool {
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = core::fmt::write(&mut s, args);
        self.at(s.as_bytes()).await
    }

   
    pub async fn init(&mut self) {
        info!("=== RYLR998 init ===");
        self.at(b"AT\r\n").await;
        self.apply_freq().await;
        self.apply_network().await;
        self.apply_parameter().await;
        self.apply_address().await;
        self.apply_power().await;
        self.at(b"AT+MODE=0\r\n").await;
        info!(
            "Ready | addr={} peer={} band={} net={} sf={} pwr={}",
            self.config.address,
            self.config.peer,
            self.config.band,
            self.config.network_id,
            self.config.sf,
            self.config.power,
        );
    }


    pub async fn set_freq(&mut self, hz: u32) -> bool {
        self.config.band = hz;
        self.apply_freq().await
    }

    pub async fn set_address(&mut self, addr: u8) -> bool {
        self.config.address = addr;
        self.apply_address().await
    }

    pub async fn set_peer(&mut self, addr: u8) {
        self.config.peer = addr; 
    }

    pub async fn set_network(&mut self, id: u8) -> bool {
        self.config.network_id = id;
        self.apply_network().await
    }

    pub async fn set_power(&mut self, dbm: u8) -> bool {
        self.config.power = dbm;
        self.apply_power().await
    }

    pub async fn set_sf(&mut self, sf: u8) -> bool {
        self.config.sf = sf;
        self.apply_parameter().await
    }


async fn apply_freq(&mut self) -> bool {
    let band = self.config.band;
    self.at_fmt(format_args!("AT+BAND={}\r\n", band)).await
}

async fn apply_network(&mut self) -> bool {
    let id = self.config.network_id;
    self.at_fmt(format_args!("AT+NETWORKID={}\r\n", id)).await
}

async fn apply_address(&mut self) -> bool {
    let addr = self.config.address;
    self.at_fmt(format_args!("AT+ADDRESS={}\r\n", addr)).await
}

async fn apply_power(&mut self) -> bool {
    let pwr = self.config.power;
    self.at_fmt(format_args!("AT+CRFOP={}\r\n", pwr)).await
}

async fn apply_parameter(&mut self) -> bool {
    let sf       = self.config.sf;
    let bw       = self.config.bw;
    let cr       = self.config.cr;
    let preamble = self.config.preamble;
    self.at_fmt(format_args!(
        "AT+PARAMETER={},{},{},{}\r\n",
        sf, bw, cr, preamble,
    )).await
}

    pub async fn send(&mut self, data: impl AsRef<[u8]>) -> bool {
        let data = data.as_ref();
        let peer = self.config.peer;
        self.at_fmt(format_args!(
            "AT+SEND={},{},{}\r\n",
            peer,
            data.len(),
            core::str::from_utf8(data).unwrap_or("")
        )).await
    }
    
    pub async fn send_to(&mut self, addr: u8, data: impl AsRef<[u8]>) -> bool {
        let data = data.as_ref();
        self.at_fmt(format_args!(
            "AT+SEND={},{},{}\r\n",
            addr,
            data.len(),
            core::str::from_utf8(data).unwrap_or("")
        )).await
    }
    pub async fn recv(&mut self) -> Option<LoraPacket> {
        let mut line: heapless::String<256> = heapless::String::new();

        loop {
            if let Ok(n) = self.uart.read(&mut self.buf).await {
                for &byte in &self.buf[..n] {
                    if byte == b'\n' {
                        let s = line.trim_matches('\r');
                        if s.starts_with("+RCV=") {
                            let pkt = Self::parse_rcv(s);
                            line.clear();
                            return pkt;
                        }
                        line.clear();
                    } else {
                        let _ = line.push(byte as char);
                    }
                }
            }
        }
    }

    fn parse_rcv(s: &str) -> Option<LoraPacket> {
        let s = s.strip_prefix("+RCV=")?;
        let mut it = s.splitn(5, ',');

        let from = it.next()?.trim().parse::<u8>().ok()?;
        let _len = it.next()?;
        let raw  = it.next()?;
        let rssi = it.next()?.trim().parse::<i16>().ok()?;
        let snr  = it.next()?.trim().parse::<i8>().ok()?;

        let mut data: heapless::String<64> = heapless::String::new();
        let _ = data.push_str(raw);

        Some(LoraPacket { from, data, rssi, snr })
    }
}

type LoraUart   = Uart<'static, esp_hal::Async>;
type LoraDriver = Lora<LoraUart>;
type SharedLora = Mutex<CriticalSectionRawMutex, LoraDriver>;
static LORA: StaticCell<SharedLora> = StaticCell::new();

#[embassy_executor::task]
async fn receiver_task(lora: &'static SharedLora,mut piezo: Output<'static>) {
    loop {
        let pkt = lora.lock().await.recv().await;
        if let Some(p) = pkt {
            info!("PKT from={} rssi={}dBm snr={} data={}", p.from, p.rssi, p.snr, p.data.as_str());
            piezo.toggle();
        }
    }
}

#[embassy_executor::task]
async fn sender_task(lora: &'static SharedLora) {
    let mut count: u32 = 0;

    loop {
        Timer::after(SEND_INTERVAL).await;
        count += 1;

        let mut payload: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::write(&mut payload, format_args!("12345678912456783|25PKT:{}", count));

        let ok = lora.lock().await.send(payload.as_bytes()).await;
        if ok {
            info!("[TX] #{} ✓", count);
        } else {
            info!("[TX] #{} ✗", count);
        }
    }
}



#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let uart = Uart::new(peripherals.UART2, Config::default().with_baudrate(115200))
        .unwrap()
        .with_rx(peripherals.GPIO4)
        .with_tx(peripherals.GPIO23)
        .into_async();

    let mut piezo: Output<'static> = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, OutputConfig::default());
    let lora_config = LoraConfig::default_eu()
        .freq(915000000)
        .network(18)
        .power(22)
        .sf(9);

    let mut driver = Lora::new(uart, lora_config);
    driver.init().await;


    let lora: &'static SharedLora = LORA.init(Mutex::new(driver));

    if IS_A_RECEIVER {
        spawner.spawn(receiver_task(lora,piezo)).unwrap();
    } else {
        spawner.spawn(sender_task(lora)).unwrap();
    }
}
