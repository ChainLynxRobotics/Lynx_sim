class Joe {
    private final long counterPtr;
    public Joe() {
        this.counterPtr = newCounter();
    }
    private static native long newCounter();
    private static native void incrementCounter(long ptr);
    private static native int counterValue(long ptr);
    private static native void deallocCounter(long ptr);

    static {
        System.loadLibrary(
            "swerve_sim_3d"
        );
    }

    public static void main(String[] args) {
        var joe = new Joe();
        System.out.println(counterValue(joe.counterPtr));
        incrementCounter(joe.counterPtr);
        System.out.println(counterValue(joe.counterPtr));
        incrementCounter(joe.counterPtr);
        System.out.println(counterValue(joe.counterPtr));
        incrementCounter(joe.counterPtr);
        incrementCounter(joe.counterPtr);
        System.out.println(counterValue(joe.counterPtr));
    }
}