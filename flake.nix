{
  description = "pio-simulator — RP2040/RP2350 PIO functional simulator + assembler";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAllSystems = f:
        nixpkgs.lib.genAttrs systems (system: f (import nixpkgs { inherit system; }));
    in {
      # `nix develop` — a working clang-21 toolchain on PATH. Built on the
      # llvmPackages_21 stdenv so the wrapped clang wires up crt/libc and its
      # resource dir: sanitizer, libFuzzer and source-coverage binaries link,
      # and clang-tidy finds the system headers (unlike a bare store-path clang).
      devShells = forAllSystems (pkgs: {
        default = (pkgs.mkShell.override { stdenv = pkgs.llvmPackages_21.stdenv; }) {
          packages = with pkgs; [
            llvmPackages_21.clang-tools # clang-format / clang-tidy (21), matching CI
            llvm_21 # llvm-cov, llvm-profdata (coverage)
            cmake
            ninja
            gcc # the CI matrix also builds with gcc
            gcovr
            git
          ];
          shellHook = ''
            echo "pio-simulator dev shell: $(clang --version | head -1)"
          '';
        };
      });
    };
}
