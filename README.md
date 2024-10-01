# Canonical-Toolchain
A template for the TERMINUS program's various RP2040/RP2350 architecture based RTIC applications


## This is largely a slimmed down version of the rp-hal examples.
Link: https://github.com/rp-rs/rp-hal/tree/main


To get started you will want the pico-sdk and picotool software installed on your development machine.

Pico-SDK (Just download): https://github.com/raspberrypi/pico-sdk \
Picotool (Download and install): https://github.com/raspberrypi/picotool \
Please see Appendix B, in the following document to build and install picotool: https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf

There are a number of ways to load your executable onto the Pico, feel free to take a look at it in the following configuration file. 
https://github.com/rp-rs/rp-hal/blob/main/rp235x-hal-examples/.cargo/config.toml.

After you've done those steps, you will need to clone the rp-hal repository. Currently the main branch of the rp-hal repository is the only one with a rp2350 hal. 

That is here: https://github.com/rp-rs/rp-hal/tree/main.

Our Cargo.toml file currently prefers this to be in the same parent folder as the Canonical-Toolchain.

After you have cloned that, you should be able to run our software!

Cheers!
