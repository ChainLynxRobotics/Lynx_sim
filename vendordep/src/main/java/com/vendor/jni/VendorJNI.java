package com.vendor.jni;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Demo class for loading the driver via JNI.
 */
public class VendorJNI {
  static boolean libraryLoaded = false;
  private static final String libraryName = "swerve_sim_3d";

  /**
   * Helper class for determining whether or not to load the driver on static initialization.
   */
  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(false);

    /**
     * Get whether to load the driver on static init.
     * @return true if the driver will load on static init
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Set whether to load the driver on static init.
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      System.loadLibrary(libraryName);
      libraryLoaded = true;
    }
  }

  /**
   * Force load the library.
   */
  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary(libraryName);
    libraryLoaded = true;
  }

  public static native long newCounter();
  public static native void incrementCounter(long ptr);
  public static native int counterValue(long ptr);
  public static native void deallocCounter(long ptr);
}
