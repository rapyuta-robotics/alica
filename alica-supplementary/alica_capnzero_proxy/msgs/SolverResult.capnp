@0xfcdd9ec652f24502;

struct SolverResult {
    senderId @0 :List(UInt8);
    vars @1 :List(SolverVar);
}

struct SolverVar {
    id @0 :Int64;
    value @1 :List(UInt8);
}