use bin_packets::{packets::CommandPacket, ConnectionTest};
use bincode::{config::standard, error::DecodeError};
use canonical_toolchain::{print, println};
use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Read;
use fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

use crate::{
    app::{incoming_packet_handler, *},
    communications::link_layer::{LinkLayerPayload, LinkPacket},
    Mono,
};

pub async fn heartbeat(ctx: heartbeat::Context<'_>) {
    loop {
        _ = ctx.local.led.toggle();

        Mono::delay(300_u64.millis()).await;
    }
}

pub async fn incoming_packet_handler(mut ctx: incoming_packet_handler::Context<'_>) {
    let mut connection_test_sequence: u16 = 0;
    let mut connection_test_start = Mono::now();
    loop {
        let buffer = ctx
            .shared
            .radio_link
            .lock(|radio| radio.device.clone_buffer());

        let decode: Result<(LinkPacket, usize), bincode::error::DecodeError> =
            bincode::decode_from_slice(&buffer, standard());

        match decode {
            Err(e) => match e {
                #[allow(unused_variables)]
                DecodeError::UnexpectedVariant {
                    type_name,
                    allowed,
                    found,
                } => {
                    // Clear the buffer
                    ctx.shared.radio_link.lock(|radio| radio.device.clear());
                }

                #[allow(unused_variables)]
                DecodeError::UnexpectedEnd { additional } => {
                    // Nothing to do here
                }

                // These decoding errors cause us to pop the front of the buffer to remove the character
                #[allow(unused_variables)]
                DecodeError::InvalidIntegerType { expected, found } => {
                    let mut buffer = [0u8; 1];
                    ctx.shared
                        .radio_link
                        .lock(|radio| radio.device.read(&mut buffer).ok());
                }

                _ => {
                    let mut buffer = alloc::string::String::new();
                    write!(buffer, "Error decoding packet: {:#?}", e).ok();
                    for c in buffer.chars() {
                        print!(ctx, "{}", c);
                        Mono::delay(1_u64.millis()).await;
                    }
                    println!(ctx, "");
                }
            },

            Ok(packet_wrapper) => {
                let packet = packet_wrapper.0;
                let read = packet_wrapper.1;
                // Drop the read bytes
                ctx.shared
                    .radio_link
                    .lock(|radio| radio.device.drop_bytes(read));

                // Uncomment the below if you think you made a mistake in handling

                // let mut buffer_heapless_stirng: alloc::string::String =
                //     alloc::string::String::new();
                // write!(buffer_heapless_stirng, "{:#?}", packet).ok();
                // for char in buffer_heapless_stirng.chars() {
                //     print!(ctx, "{}", char);
                //     Mono::delay(1_u64.millis()).await;
                // }
                // println!(ctx, "\n");

                // Check the checksum, if it fails, the packet is bad, we should continue
                // and clear the buffer
                if !packet.verify_checksum() {
                    ctx.shared.radio_link.lock(|radio| radio.device.clear());
                    println!(ctx, "Bad Packet, checksum failure");
                    continue;
                }

                match packet.payload {
                    LinkLayerPayload::Payload(app_packet) => match app_packet {
                        bin_packets::ApplicationPacket::Command(command) => match command {
                            // Connection test sequence
                            CommandPacket::ConnectionTest(connection) => match connection {
                                ConnectionTest::Start => {
                                    connection_test_sequence = 0;
                                    connection_test_start = Mono::now();
                                }

                                ConnectionTest::Sequence(seq) => {
                                    connection_test_sequence += 1;
                                    println!(ctx, "Received Connection Test Sequence: {}", seq);
                                }

                                ConnectionTest::End => {
                                    println!(ctx, "Received Connection Test End");

                                    let percentage_recieved =
                                        (connection_test_sequence as f32 / 256.0) * 100.0;
                                    println!(
                                        ctx,
                                        "Received {}% of the connection test sequence",
                                        percentage_recieved
                                    );

                                    let elapsed = Mono::now() - connection_test_start;
                                    println!(ctx, "Elapsed Time: {}ms", elapsed.to_millis());
                                }
                            },
                            _ => {}
                        },

                        _ => {
                            let mut buffer_heapless_stirng: alloc::string::String =
                                alloc::string::String::new();
                            write!(buffer_heapless_stirng, "{:#?}", packet).ok();
                            for char in buffer_heapless_stirng.chars() {
                                print!(ctx, "{}", char);
                                Mono::delay(1_u64.millis()).await;
                            }
                            println!(ctx, "\n");
                        }
                    },

                    _ => {}
                }
            }
        }

        Mono::delay(10_u64.millis()).await;
    }
}
