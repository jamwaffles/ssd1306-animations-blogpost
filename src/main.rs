#![no_std]
#![feature(const_fn)]
#![feature(proc_macro)]
#![feature(used)]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_rtfm_macros;
extern crate embedded_graphics;
extern crate panic_abort;
extern crate ssd1306;
extern crate stm32f103xx_hal as hal;

use cortex_m_rtfm_macros::app;
use embedded_graphics::image::Image1BPP;
use embedded_graphics::prelude::*;
use hal::delay::Delay;
use hal::gpio::gpiob::{PB8, PB9};
use hal::gpio::{Alternate, OpenDrain};
use hal::i2c::{DutyCycle, I2c, Mode};
use hal::prelude::*;
use hal::stm32f103xx::I2C1;
use hal::timer::{self, Timer};
use rtfm::Threshold;
use ssd1306::prelude::*;
use ssd1306::Builder;

pub type OledDisplay =
    GraphicsMode<I2cInterface<I2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>>;

type Position = (u32, u32);

const SQUARE_SIZE: u32 = 24;

// Tasks and resources
app! {
    device: hal::stm32f103xx,

    resources: {
        static DISP: OledDisplay;
        static IMAGE: Image1BPP<'static>;
        static POS: Position = (0, 0);
        static XDELTA: i32 = 1;
        static YDELTA: i32 = 1;
    },

    tasks: {
        SYS_TICK: {
            path: tick,
            resources: [DISP, IMAGE, POS, XDELTA, YDELTA],
        },
    },
}

fn init(p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = p.device.AFIO.constrain(&mut rcc.apb2);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.apb2);
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = I2c::i2c1(
        p.device.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000,
            duty_cycle: DutyCycle::Ratio1to1,
        },
        clocks,
        &mut rcc.apb1,
    );

    let delay = Delay::new(p.core.SYST, clocks);
    Timer::syst(delay.free(), 30.hz(), clocks).listen(timer::Event::Update);

    let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

    disp.init().unwrap();
    disp.flush().unwrap();

    let image = Image1BPP::new(include_bytes!("../rust.raw"), SQUARE_SIZE, SQUARE_SIZE);

    init::LateResources {
        DISP: disp,
        IMAGE: image,
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    let (width, height) = r.DISP.get_dimensions();

    r.DISP.clear();

    r.DISP
        .draw(r.IMAGE.translate((r.POS.0, r.POS.1)).into_iter());

    r.DISP.flush().unwrap();

    // Detect left/right edge collisions
    if r.POS.0 as i32 + *r.XDELTA == (width - SQUARE_SIZE as u8 - 1) as i32
        || r.POS.0 as i32 + *r.XDELTA == 0
    {
        *r.XDELTA *= -1;
    }

    // Detect top/bottom edge collisions
    if r.POS.1 as i32 + *r.YDELTA == (height - SQUARE_SIZE as u8 - 1) as i32
        || r.POS.1 as i32 + *r.YDELTA == 0
    {
        *r.YDELTA *= -1;
    }

    *r.POS = (
        (r.POS.0 as i32 + *r.XDELTA) as u32,
        (r.POS.1 as i32 + *r.YDELTA) as u32,
    );
}
