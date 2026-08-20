{
  description = "A flake to provide dev shells for this project";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      fenix,
    }:
    {
      devShells = builtins.mapAttrs (
        system: pkgs2:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ fenix.overlays.default ];
          };
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              bacon
              (pkgs.fenix.latest.withComponents [
                "cargo"
                "clippy"
                "rust-src"
                "rustc"
                "rustfmt"
              ])
              temurin-bin-25
              self.packages.${system}.gplusplus14
              gradle_9
            ];

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (
              with pkgs;
              [
                wayland
                libxkbcommon
                vulkan-loader
              ]
            );
            shellHook = "zsh";
          };
        }
      ) nixpkgs.legacyPackages;
      formatter = builtins.mapAttrs (system: pkgs: pkgs.nixfmt-tree) nixpkgs.legacyPackages;

      packages = builtins.mapAttrs (system: pkgs: {
        gplusplus14 = pkgs.callPackage ./g++-14.nix { };
      }) nixpkgs.legacyPackages;
    };
}
