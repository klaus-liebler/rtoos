#include "rtos/opcua/application.hpp"

namespace rtos::opcua {

namespace {
ApplicationIdentity g_identity;
String g_namespaces[] = {String("http://opcfoundation.org/UA/"), String(ApplicationIdentity{}.applicationUri)};
String g_servers[] = {String(ApplicationIdentity{}.applicationUri)};
} // namespace

void SetApplicationIdentity(const ApplicationIdentity &identity) noexcept {
    g_identity = identity;
    g_namespaces[1] = String(identity.applicationUri);
    g_servers[0] = String(identity.applicationUri);
}

const ApplicationIdentity &Application() noexcept { return g_identity; }

Variant ReadNamespaceArray(void *) { return Variant::Of(StringArray{g_namespaces}); }
Variant ReadServerArray(void *) { return Variant::Of(StringArray{g_servers}); }

}  // namespace rtos::opcua
