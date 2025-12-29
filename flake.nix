{
  description = "RMK - Rust keyboard firmware";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    crane.url = "github:ipetkov/crane";

    flake-utils.url = "github:numtide/flake-utils";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    self,
    nixpkgs,
    crane,
    flake-utils,
    rust-overlay,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [(import rust-overlay)];
      };

      # Embedded targets from rust-toolchain.toml
      embeddedTargets = [
        "thumbv7em-none-eabi"
        "thumbv7m-none-eabi"
        "thumbv6m-none-eabi"
        "thumbv7em-none-eabihf"
        "thumbv8m.main-none-eabihf"
        "riscv32imc-unknown-none-elf"
        "riscv32imac-unknown-none-elf"
      ];

      # Rust toolchain matching rust-toolchain.toml
      rustToolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = ["rust-src" "rustfmt" "llvm-tools" "clippy"];
        targets = embeddedTargets;
      };

      craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;

      # Common source filtering for Rust builds
      src = craneLib.cleanCargoSource ./.;

      # Common arguments for crane builds
      commonArgs = {
        inherit src;

        # Build with std feature for testing on host
        cargoExtraArgs = "--package rmk --features std,log";

        buildInputs = with pkgs;
          [
            pkg-config
            openssl
          ]
          ++ pkgs.lib.optionals pkgs.stdenv.isDarwin [
            pkgs.libiconv
            pkgs.darwin.apple_sdk.frameworks.Security
            pkgs.darwin.apple_sdk.frameworks.SystemConfiguration
          ];

        nativeBuildInputs = with pkgs; [
          pkg-config
        ];
      };

      # Build dependencies separately for caching
      cargoArtifacts = craneLib.buildDepsOnly commonArgs;

      # Build the rmk crate (with std for host testing)
      rmk = craneLib.buildPackage (commonArgs
        // {
          inherit cargoArtifacts;
          doCheck = true;
        });

      # Clippy check
      rmkClippy = craneLib.cargoClippy (commonArgs
        // {
          inherit cargoArtifacts;
          cargoClippyExtraArgs = "--all-targets -- --deny warnings";
        });

      # Format check
      rmkFmt = craneLib.cargoFmt {
        inherit src;
      };

      # Documentation
      rmkDoc = craneLib.cargoDoc (commonArgs
        // {
          inherit cargoArtifacts;
        });
    in {
      checks = {
        inherit rmk rmkClippy rmkFmt;
      };

      packages = {
        default = rmk;
        doc = rmkDoc;
      };

      devShells.default = craneLib.devShell {
        checks = self.checks.${system};

        packages = with pkgs; [
          # Rust toolchain is provided by craneLib.devShell

          # Embedded development tools
          probe-rs-tools
          flip-link
          elf2uf2-rs
          cargo-binutils

          # Build essentials
          pkg-config
          openssl

          # Useful development tools
          cargo-watch
          cargo-expand
          cargo-make

          # Documentation
          mdbook
        ];

        shellHook = ''
          echo "RMK development shell"
          echo "Available targets: ${builtins.concatStringsSep ", " embeddedTargets}"
          echo ""
          echo "Useful commands:"
          echo "  cargo build --release          - Build for host (testing)"
          echo "  cargo build --target <target>  - Build for embedded target"
          echo "  probe-rs run --chip <chip>     - Flash and run firmware"
          echo ""
        '';
      };
    });
}
