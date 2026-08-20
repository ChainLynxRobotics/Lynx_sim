package com.vendor.jni;

import org.junit.jupiter.api.Test;

public class VendorJNITest {
  @Test
  void jniLinkTest() {
    VendorJNI.forceLoad();
    // Test to verify that the JNI test link works correctly.
    long counterPtr = VendorJNI.newCounter();
    VendorJNI.counterValue(counterPtr);
    VendorJNI.incrementCounter(counterPtr);
    VendorJNI.incrementCounter(counterPtr);
    VendorJNI.deallocCounter(counterPtr);
  }
}