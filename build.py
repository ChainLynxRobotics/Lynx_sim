import os
import subprocess
import sys
import zipfile

# TODO: Load from cargo metadata
VERSION = "2027.0.0"

def run(*cmd):
  env = os.environ.copy()
  sp = subprocess.Popen(cmd, env=env)
  sp.wait()

TRIPLE_LOOKUP = {
  ("windowsx86-64"): {
    'triple': "x86_64-pc-windows-msvc",
    'path': 'windows/x86-64'
  },
  ("windowsarm64"): {
    'triple': "aarch64-pc-windows-msvc",
    'path': 'windows/arm64'
  },
  ("osxuniversal"): {
    'triple': "aarch64-apple-darwin",
    'path': 'osx/universal'
  },
  ("linuxx86-64"): {
    'triple': "x86_64-unknown-linux-gnu",
    'path': 'linux/x86-64'
  },
  ("linuxarm64"): {
    'triple': "aarch64-unknown-linux-gnu",
    'path': 'linux/arm64'
  },
}

def build(platform):
  details = TRIPLE_LOOKUP.get(platform)
  triple = details['triple']

  if triple is None:
    print("No Triple found for {}".format(platform), file=sys.stderr)
    exit(1)

  run("cargo", "build", "--target={}".format(triple), "--features", "distribution")
  run("cargo", "build", "--profile", "release-full", "--target={}".format(triple), "--features", "distribution")

  # OSX builds universal by building both and then merging
  if platform == "osxuniversal":
    run("cargo", "build", "--target=x86_64-apple-darwin", "--features", "distribution")
    run("cargo", "build", "--profile", "release-full", "--target=x86_64-apple-darwin", "--features", "distribution")

    for mode in ["debug", "release-full"]:
      mode_dir = f"target/universal-apple-darwin/{mode}"
      try:
        os.makedirs(mode_dir)
      except FileExistsError:
        pass
      output_file = f"{mode_dir}/libswerve_sim_3d.dylib"
      intel_file = f"target/x86_64-apple-darwin/{mode}/libswerve_sim_3d.dylib"
      arm_file = f"target/aarch64-apple-darwin/{mode}/libswerve_sim_3d.dylib"
      run("lipo", "-create", "-output", output_file, intel_file, arm_file)
      triple = "universal-apple-darwin"

  # Zip it up for maven
  package = "com/chainlynx8248"
  identifier = "libswerve_sim_3d"
  classifierBase = platform
  outdir = f"target/zips/{package}/{identifier}/{VERSION}"

  try:
    os.makedirs(outdir)
  except FileExistsError:
    pass

  # Then everything else
  files = {}
  if "windows" in platform:
    files = {
      'shared': { 'swerve_sim_3d.dll': 'swerve_sim_3d.dll', 'swerve_sim_3d.dll.lib': 'swerve_sim_3d.lib', 'swerve_sim_3d.pdb': 'swerve_sim_3d.pdb' },
      'static': { 'swerve_sim_3d.lib': 'swerve_sim_3d.lib' }
    }
  elif "osx" in platform:
    files = {
      'shared': { 'libswerve_sim_3d.dylib': 'libswerve_sim_3d.dylib' }
    }
  else:
    files = {
      'shared': { 'libswerve_sim_3d.so': 'libswerve_sim_3d.so' },
      'static': { 'libswerve_sim_3d.a': 'libswerve_sim_3d.a' }
    }

  for (linkage, linkageClassifier) in [("shared", ""), ("static", "static")]:
    for (mode, classifier) in [("debug", f"{classifierBase}{linkageClassifier}debug"), ("release-full", f"{classifierBase}{linkageClassifier}")]:
      if linkage in files:
        with zipfile.ZipFile(f"{outdir}/{identifier}-{VERSION}-{classifier}.zip", "w") as zf:
          for (fkey, fname) in files[linkage].items():
            zf.write(f"target/{triple}/{mode}/{fkey}", f"{details['path']}/{linkage}/{fname}")

  # And lastly, the .pom
  with open(f"{outdir}/{identifier}-{VERSION}.pom", "w") as f:
    f.write("""<?xml version="1.0" encoding="UTF-8"?>
<project xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd" xmlns="http://maven.apache.org/POM/4.0.0"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <modelVersion>4.0.0</modelVersion>
  <groupId>{}</groupId>
  <artifactId>{}</artifactId>
  <version>{}</version>
  <packaging>pom</packaging>
</project>""".format(package.replace("/", "."), identifier, VERSION))

build(sys.argv[1])