class Joe {
    private static native String hello(String input);

    static {
        System.loadLibrary(
            "swerve_sim_3d"
        );
    }

    public static void main(String[] args) {
        String output = Joe.hello("World");
        System.out.println(output);
    }
}