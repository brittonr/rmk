use quote::{format_ident, quote};
use rmk_config::{ChipModel, ChipSeries, Pmw3360Config};

use super::Initializer;

/// Expand PMW3360 device configuration.
/// Returns (device initializers, processor initializers)
pub(crate) fn expand_pmw3360_device(
    pmw3360_config: Vec<Pmw3360Config>,
    chip: &ChipModel,
) -> (Vec<Initializer>, Vec<Initializer>) {
    if pmw3360_config.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // PMW3360 is only supported on nRF52 and RP2040
    match chip.series {
        ChipSeries::Nrf52 | ChipSeries::Rp2040 => {}
        _ => {
            panic!("PMW3360 is only supported on nRF52 and RP2040 chips");
        }
    }

    let mut device_initializers = vec![];
    let mut processor_initializers = vec![];

    for (idx, sensor) in pmw3360_config.iter().enumerate() {
        let sensor_name = if sensor.name.is_empty() {
            format!("pmw3360_{}", idx)
        } else {
            sensor.name.clone()
        };

        let device_ident = format_ident!("{}_device", sensor_name);
        let processor_ident = format_ident!("{}_processor", sensor_name);

        // Generate pin initialization
        let spi = &sensor.spi;
        let sck_ident = format_ident!("{}", spi.sck);
        let mosi_ident = format_ident!("{}", spi.mosi);
        let miso_ident = format_ident!("{}", spi.miso);
        let cs_ident = format_ident!("{}", spi.cs.as_ref().expect("pmw3360 requires `cs` in spi config"));

        // Generate config values
        let res_cpi: u16 = sensor.cpi.unwrap_or(1600);
        let rot_trans_angle: i8 = sensor.rot_trans_angle;
        let liftoff_dist: u8 = sensor.liftoff_dist;
        let invert_x = sensor.invert_x;
        let invert_y = sensor.invert_y;
        let swap_xy = sensor.swap_xy;

        // Generate motion pin initialization (optional)
        let motion_pin_init = if let Some(motion_pin) = &sensor.motion {
            let motion_ident = format_ident!("{}", motion_pin);
            match chip.series {
                ChipSeries::Nrf52 => quote! {
                    Some(::embassy_nrf::gpio::Input::new(p.#motion_ident, ::embassy_nrf::gpio::Pull::Up))
                },
                ChipSeries::Rp2040 => quote! {
                    Some(::embassy_rp::gpio::Input::new(p.#motion_ident, ::embassy_rp::gpio::Pull::Up))
                },
                _ => unreachable!(),
            }
        } else {
            match chip.series {
                ChipSeries::Nrf52 => quote! {
                    None::<::embassy_nrf::gpio::Input<'static>>
                },
                ChipSeries::Rp2040 => quote! {
                    None::<::embassy_rp::gpio::Input<'static>>
                },
                _ => unreachable!(),
            }
        };

        // Generate SPI instance identifier
        let spi_instance = format_ident!("{}", spi.instance.to_uppercase());

        // Generate device initialization based on chip series
        let device_init = match chip.series {
            ChipSeries::Nrf52 => quote! {
                let mut #device_ident = {
                    use ::embassy_nrf::gpio::{Output, Level, OutputDrive};
                    use ::embassy_nrf::spim::{self, Spim};
                    use ::rmk::input_device::pmw3360::{Pmw3360Config, Pmw3360Device};

                    let mut spi_config = spim::Config::default();
                    spi_config.frequency = spim::Frequency::M2;
                    spi_config.mode = spim::MODE_3;

                    let spi = Spim::new_txrx(
                        p.#spi_instance,
                        ::embassy_nrf::Irqs,
                        p.#sck_ident,
                        p.#miso_ident,
                        p.#mosi_ident,
                        spi_config,
                    );
                    let cs = Output::new(p.#cs_ident, Level::High, OutputDrive::Standard);
                    let motion = #motion_pin_init;

                    let config = Pmw3360Config {
                        res_cpi: #res_cpi,
                        rot_trans_angle: #rot_trans_angle,
                        liftoff_dist: #liftoff_dist,
                        invert_x: #invert_x,
                        invert_y: #invert_y,
                        swap_xy: #swap_xy,
                    };

                    Pmw3360Device::new(spi, cs, motion, config)
                };
            },
            ChipSeries::Rp2040 => {
                // Use DMA channels based on sensor index to avoid conflicts
                let dma_tx = format_ident!("DMA_CH{}", 2 + idx * 2);
                let dma_rx = format_ident!("DMA_CH{}", 3 + idx * 2);
                quote! {
                    let mut #device_ident = {
                        use ::embassy_rp::gpio::{Output, Level};
                        use ::embassy_rp::spi::{self, Spi, Polarity, Phase};
                        use ::rmk::input_device::pmw3360::{Pmw3360Config, Pmw3360Device};

                        let mut spi_config = spi::Config::default();
                        spi_config.frequency = 2_000_000;
                        spi_config.polarity = Polarity::IdleHigh;
                        spi_config.phase = Phase::CaptureOnSecondTransition;

                        let spi = Spi::new(
                            p.#spi_instance,
                            p.#sck_ident,
                            p.#mosi_ident,
                            p.#miso_ident,
                            p.#dma_tx,
                            p.#dma_rx,
                            spi_config,
                        );
                        let cs = Output::new(p.#cs_ident, Level::High);
                        let motion = #motion_pin_init;

                        let config = Pmw3360Config {
                            res_cpi: #res_cpi,
                            rot_trans_angle: #rot_trans_angle,
                            liftoff_dist: #liftoff_dist,
                            invert_x: #invert_x,
                            invert_y: #invert_y,
                            swap_xy: #swap_xy,
                        };

                        Pmw3360Device::new(spi, cs, motion, config)
                    };
                }
            },
            _ => unreachable!(),
        };

        device_initializers.push(Initializer {
            initializer: device_init,
            var_name: device_ident,
        });

        // Generate processor initialization (use Pmw3610Processor which works for both)
        let processor_init = if let Some(auto_mouse_layer) = sensor.auto_mouse_layer {
            let timeout_ms = sensor.auto_mouse_timeout_ms;
            quote! {
                let mut #processor_ident = ::rmk::input_device::pmw3610::Pmw3610Processor::with_auto_mouse(
                    &keymap,
                    #auto_mouse_layer,
                    #timeout_ms,
                );
            }
        } else {
            quote! {
                let mut #processor_ident = ::rmk::input_device::pmw3610::Pmw3610Processor::new(&keymap);
            }
        };

        processor_initializers.push(Initializer {
            initializer: processor_init,
            var_name: processor_ident,
        });
    }

    (device_initializers, processor_initializers)
}
