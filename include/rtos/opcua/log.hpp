#pragma once
// Debug trace points of the OPC UA library (formerly the FCU firmware's printf-style
// log_debug). rtoos has no printf on the target (newlib's could pull in the heap):
// they compile to nothing; the server counts the events that matter (opcua_server.hpp).
#define log_debug(...) ((void)0)
