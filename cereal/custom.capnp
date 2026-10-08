using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct DpControlsState @0x81c2f05a394cf4af {
  alkaActive @0 :Bool;
}

struct ModelExt @0xaedffd8f31e7b55d {
  leftEdgeDetected @0 :Bool;
  rightEdgeDetected @1 :Bool;
}

struct ModelManagerSP @0xf35cc4560bbf6ec2 {
  activeBundle @0 :ModelBundle;
  selectedBundle @1 :ModelBundle;
  availableBundles @2 :List(ModelBundle);

  struct DownloadUri {
    uri @0 :Text;
    sha256 @1 :Text;
  }

  enum DownloadStatus {
    notDownloading @0;
    downloading @1;
    downloaded @2;
    cached @3;
    failed @4;
  }

  struct DownloadProgress {
    status @0 :DownloadStatus;
    progress @1 :Float32;
    eta @2 :UInt32;
  }

  struct Artifact {
    fileName @0 :Text;
    downloadUri @1 :DownloadUri;
    downloadProgress @2 :DownloadProgress;
  }

  struct Model {
    type @0 :Type;
    artifact @1 :Artifact;
    metadata @2 :Artifact;

    enum Type {
      supercombo @0;
      navigation @1;
      vision @2;
      policy @3;
    }
  }

  enum Runner {
    snpe @0;
    tinygrad @1;
    stock @2;
  }

  struct Override {
    key @0 :Text;
    value @1 :Text;
  }

  struct ModelBundle {
    index @0 :UInt32;
    internalName @1 :Text;
    displayName @2 :Text;
    models @3 :List(Model);
    status @4 :DownloadStatus;
    generation @5 :UInt32;
    environment @6 :Text;
    runner @7 :Runner;
    is20hz @8 :Bool;
    ref @9 :Text;
    minimumSelectorVersion @10 :UInt32;
    overrides @11 :List(Override);
  }
}

struct AmapNavi @0xda96579883444c35 {
	leftBlind @0 : Int32;
	rightBlind @1 : Int32;
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct MR76State @0xcb9fd56c7057593a {
  valid @0 :Bool;

  radarStateValid @1 :Bool;
  statusValid @2 :Bool;
  objectDataValid @3 :Bool;

  nvmReadStatus @4 :UInt8;
  nvmWriteStatus @5 :UInt8;

  maxDistance @6 :Float32;
  radarPower @7 :UInt8;
  sensorId @8 :UInt8;
  sortIndex @9 :UInt8;

  outputType @10 :UInt8;
  qualityInfo @11 :Bool;
  extInfo @12 :Bool;

  canBaudRate @13 :UInt8;
  interfaceType @14 :UInt8;
  rcsThreshold @15 :UInt8;
  calibrationEnabled @16 :UInt8;

  numObjects @17 :UInt16;
  measCount @18 :UInt32;
  interfaceVersion @19 :UInt8;

  objectCount @20 :UInt16;

  objects @21 :List(Target);

  lastUpdateMonoTime @22 :UInt64;
}

struct Target {
  id @0 :UInt8;

  distLong @1 :Float32;
  distLat @2 :Float32;

  vRelLong @3 :Float32;
  vRelLat @4 :Float32;

  dynProp @5 :UInt8;
  targetClass @6 :UInt8;

  rcs @7 :Float32;
  distance @8 :Float32;

  lastUpdateMonoTime @9 :UInt64;
  frameCount @10 :UInt32;
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
