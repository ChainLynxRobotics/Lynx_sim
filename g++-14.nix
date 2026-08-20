{
  gcc14,
  stdenv,
  ...
}:
stdenv.mkDerivation {
  dontUnpack = true;
  name = "g++14correctname";

  installPhase = ''
    echo $out
    mkdir $out
    mkdir $out/bin
    cp ${gcc14}/bin/g++ $out/bin/
    mv $out/bin/g++ $out/bin/g++-14
  '';
}
