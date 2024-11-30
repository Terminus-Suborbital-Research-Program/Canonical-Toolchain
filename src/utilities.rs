use core::arch::asm;

pub fn get_stack_pointer() -> usize {
    let sp: usize;
    unsafe {
        asm!("mov {}, sp", out(reg) sp);
    }
    sp
}

extern crate alloc;
use alloc::{boxed::Box, vec};
use crate::HEAP;
// Function for getting the current number of free bytes on the heap
// Does this by trying to allocate a large number of bytes and then freeing them
// It keeps increasing the number of bytes until it fails to allocate
pub fn get_free_heap() -> usize {
    HEAP.free()
}