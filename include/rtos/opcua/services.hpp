#pragma once
// The four services this server implements on the address space directly: GetEndpoints, Read,
// Write, Browse (project decision: Browse "jetzt", Subscriptions/Methods "später"). Same
// framing convention as session.hpp -- these write only TypeId + response struct into an
// already-positioned ByteWriter; connection.cpp wraps the call in
// BeginSymmetricMessage(...)/FinishSecureMessage(...).
//
// Simplifications documented individually below (all deliberate, matching the project's
// "minimal now, extensible later" scope):
//  - Browse: RequestedMaxReferencesPerNode and ReferenceTypeId/IncludeSubtypes filtering are
//    not honored -- this server's whole address space is small enough that "return
//    everything" is always correct and never exceeds a single response; add proper filtering/
//    pagination (ContinuationPoint) if/when the address space grows enough to matter.
//  - Write: only the Value attribute is writable.
#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/service_structs.hpp"

namespace rtos::opcua {

// GetEndpoints needs no active session (Part 4 5.4.4 -- callable before CreateSession, e.g. to
// discover the endpoint in the first place).
bool HandleGetEndpoints(ByteReader &requestBody, std::string_view ourEndpointUrl, ByteWriter &w);

// sessionActive: if false, the request's non-header fields are NOT parsed (harmless -- each
// message is already extracted as one complete, self-contained span by the transport layer
// before reaching here, so not fully consuming "requestBody" doesn't desync anything) and the
// response is written with ResponseHeader.serviceResult = BadSessionNotActivated and empty
// result arrays instead.
bool HandleRead(const AddressSpace &addressSpace, bool sessionActive, ByteReader &requestBody, ByteWriter &w);
bool HandleWrite(const AddressSpace &addressSpace, bool sessionActive, ByteReader &requestBody, ByteWriter &w);
bool HandleBrowse(const AddressSpace &addressSpace, bool sessionActive, ByteReader &requestBody, ByteWriter &w);

}  // namespace rtos::opcua
