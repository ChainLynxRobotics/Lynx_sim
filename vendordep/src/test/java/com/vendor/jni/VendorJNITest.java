package com.vendor.jni;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.chainlynx8248.Lynx_sim.VendorJNI;

public class VendorJNITest {
  @Test
  void jniLinkTest() {
    VendorJNI.forceLoad();
    // Test to verify that the JNI test link works correctly.
    long counterPtr = VendorJNI.newCounter();
    VendorJNI.counterValue(counterPtr);
    VendorJNI.incrementCounter(counterPtr);
    VendorJNI.incrementCounter(counterPtr);
    assertEquals(2, VendorJNI.counterValue(counterPtr));
    VendorJNI.deallocCounter(counterPtr);
  }
}