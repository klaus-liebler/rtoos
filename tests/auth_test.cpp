// SPDX-License-Identifier: MIT
//
// Host tests of the authentication building blocks (include/rtos/auth): PBKDF2 against reference
// values, the session table (tokens, timeouts, fixation, handles), the login throttle, the role
// and presence decisions, and the A/B credential store on a NOR flash fake with power failures.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "rtos/auth/access.hpp"
#include "rtos/auth/credentials.hpp"
#include "rtos/auth/pbkdf2.hpp"
#include "rtos/auth/session.hpp"
#include "rtos/auth/throttle.hpp"

namespace {

using namespace rtos::auth;

std::string hex(std::span<const std::byte> b) {
    std::string s;
    for (std::byte x : b) {
        constexpr char d[] = "0123456789abcdef";
        s += d[std::to_integer<int>(x) >> 4];
        s += d[std::to_integer<int>(x) & 15];
    }
    return s;
}
std::span<const std::byte> bytes(std::string_view s) { return std::as_bytes(std::span(s)); }

// ---- PBKDF2 ----------------------------------------------------------------------------------

// Reference values: Python's hashlib.pbkdf2_hmac('sha256', ...) - the first one is also RFC 7914 11.
TEST(Pbkdf2, MatchesReferenceValues) {
    EXPECT_EQ(hex(Pbkdf2Sha256::derive(bytes("passwd"), bytes("salt"), 1)),
              "55ac046e56e3089fec1691c22544b605f94185216dde0465e68b9d57c20dacbc");
    EXPECT_EQ(hex(Pbkdf2Sha256::derive(bytes("password"), bytes("salt"), 1)),
              "120fb6cffcf8b32c43e7225256c4f837a86548c92ccc35480805987cb70be17b");
    EXPECT_EQ(hex(Pbkdf2Sha256::derive(bytes("password"), bytes("salt"), 4096)),
              "c5e478d59288c841aa530db6845c4c8d962893a001ce4e11a4963873aa98134a");
    std::array<std::byte, 16> salt{};
    for (int i = 0; i < 16; ++i) salt[i] = std::byte(i);
    EXPECT_EQ(hex(Pbkdf2Sha256::derive(bytes("Factory-Box_2026!"), salt, 10000)),
              "b35f8bf8056a051be84236671860cc802021f005c01bfe33701f7ec05b411f4a");
}

TEST(Pbkdf2, StepsGiveTheSameResultAsOneCall) {
    Pbkdf2Sha256 p;
    ASSERT_TRUE(p.start(bytes("password"), bytes("salt"), 4096));
    int steps = 0;
    while (p.step(100)) ++steps;
    EXPECT_EQ(steps, 40);  // 4095 iterations after U_1: 41 steps, the last returns false
    ASSERT_TRUE(p.done());
    EXPECT_EQ(hex(p.result()), "c5e478d59288c841aa530db6845c4c8d962893a001ce4e11a4963873aa98134a");
}

TEST(Pbkdf2, RefusesBadParameters) {
    Pbkdf2Sha256 p;
    EXPECT_FALSE(p.start(bytes(""), bytes("salt"), 10));
    EXPECT_FALSE(p.start(bytes(std::string(65, 'x')), bytes("salt"), 10));
    EXPECT_FALSE(p.start(bytes("pw"), bytes("salt"), 0));
    EXPECT_TRUE(p.failed());
    EXPECT_FALSE(p.step(10));
}

TEST(Pbkdf2, ConstantTimeCompare) {
    const std::array<std::byte, 3> a{std::byte{1}, std::byte{2}, std::byte{3}}, b = a;
    std::array<std::byte, 3> c = a;
    c[2] = std::byte{4};
    EXPECT_TRUE(equal_constant_time(a, b));
    EXPECT_FALSE(equal_constant_time(a, c));
    EXPECT_FALSE(equal_constant_time(a, std::span(b).first(2)));
}

TEST(PasswordCheck, AcceptsTheRightPasswordOnly) {
    PasswordRecord rec{.iterations = 1000};
    rec.salt[0] = std::byte{7};
    rec.hash = Pbkdf2Sha256::derive(bytes("correct horse"), rec.salt, 1000);
    for (const auto& [pw, ok] : std::vector<std::pair<std::string, bool>>{{"correct horse", true}, {"correct hors", false},
                                                                         {"Correct horse", false}}) {
        PasswordCheck c;
        ASSERT_TRUE(c.start(pw, rec));
        while (c.step(64)) {
        }
        EXPECT_TRUE(c.done());
        EXPECT_EQ(c.matches(), ok) << pw;
    }
}

bool counting_random(std::span<std::byte> out) noexcept {
    static std::uint8_t n = 0;
    for (auto& b : out) b = std::byte(n++ * 37 + 11);
    return true;
}
bool failing_random(std::span<std::byte>) noexcept { return false; }

TEST(PasswordSetter, NewSaltEveryTimeAndVerifiable) {
    PasswordSetter s1, s2;
    ASSERT_TRUE(s1.start("new password 1", 500, counting_random));
    while (s1.step(100)) {
    }
    ASSERT_TRUE(s2.start("new password 1", 500, counting_random));
    while (s2.step(100)) {
    }
    const auto r1 = s1.record(), r2 = s2.record();
    ASSERT_TRUE(r1 && r2);
    EXPECT_NE(r1->salt, r2->salt);
    EXPECT_NE(r1->hash, r2->hash);
    PasswordCheck c;
    ASSERT_TRUE(c.start("new password 1", *r2));
    while (c.step(100)) {
    }
    EXPECT_TRUE(c.matches());
    PasswordSetter s3;
    EXPECT_FALSE(s3.start("new password 1", 500, failing_random));
}

TEST(PasswordPolicy, Rules) {
    EXPECT_TRUE(password_problem("short", "admin"));
    EXPECT_TRUE(password_problem("adminadmin", "adminadmin"));
    EXPECT_TRUE(password_problem(std::string("tab\tinside!"), "admin"));
    EXPECT_FALSE(password_problem("a long enough one", "admin"));
    EXPECT_TRUE(valid_user_name("admin"));
    EXPECT_TRUE(valid_user_name("line-1.op_2"));
    EXPECT_FALSE(valid_user_name("Admin"));
    EXPECT_FALSE(valid_user_name(""));
    EXPECT_FALSE(valid_user_name("a234567890123456"));
}

// ---- sessions --------------------------------------------------------------------------------

TEST(Session, TokensAreRandomBase64urlAndFindable) {
    SessionTable<4> t(counting_random);
    Token a{}, b{};
    const auto ha = t.create({.user = 0, .role = Role::Admin}, 1000, a);
    const auto hb = t.create({.user = 1, .role = Role::Observer}, 1000, b);
    ASSERT_TRUE(ha && hb);
    EXPECT_NE(a, b);
    for (char c : a) EXPECT_TRUE(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_');
    const auto fa = t.find({a.data(), a.size()}, 2000);
    ASSERT_TRUE(fa);
    EXPECT_EQ(*fa, *ha);
    EXPECT_EQ(t.info(*fa)->role, Role::Admin);
    // forged: one character changed, too short, empty
    Token forged = a;
    forged[10] = forged[10] == 'A' ? 'B' : 'A';
    EXPECT_FALSE(t.find({forged.data(), forged.size()}, 2000));
    EXPECT_FALSE(t.find({a.data(), 42}, 2000));
    EXPECT_FALSE(t.find("", 2000));
}

TEST(Session, NoTokenWithoutRandomNumbers) {
    SessionTable<2> t(failing_random);
    Token tok{};
    EXPECT_FALSE(t.create({.role = Role::Admin}, 0, tok));
}

TEST(Session, IdleTimeoutAndLifetime) {
    SessionTable<2> t(counting_random, {.idle_ms = 1000, .lifetime_ms = 5000});
    Token tok{};
    const auto h = t.create({.role = Role::Operator}, 0, tok);
    ASSERT_TRUE(h);
    const std::string_view s{tok.data(), tok.size()};
    EXPECT_TRUE(t.find(s, 900));    // refreshes: idle until 1900
    EXPECT_TRUE(t.touch(*h, 1800));
    EXPECT_TRUE(t.find(s, 2700));
    EXPECT_TRUE(t.find(s, 3600));
    EXPECT_TRUE(t.find(s, 4500));
    EXPECT_FALSE(t.find(s, 5000));  // lifetime over although active
    EXPECT_EQ(t.count(), 0u);
    const auto h2 = t.create({.role = Role::Operator}, 10'000, tok);
    EXPECT_FALSE(t.touch(*h2, 11'000));  // idle for exactly idle_ms
}

TEST(Session, WrapAroundOfTheMillisecondClock) {
    SessionTable<2> t(counting_random, {.idle_ms = 1000, .lifetime_ms = 10'000});
    Token tok{};
    const auto h = t.create({.role = Role::Operator}, 0xFFFF'FF00u, tok);
    EXPECT_TRUE(t.touch(*h, 0x0000'0200u));
    EXPECT_FALSE(t.touch(*h, 0x0000'0600u + 1000));
}

TEST(Session, FullTableDropsLeastRecentlyUsedAndHandlesGoStale) {
    SessionTable<2> t(counting_random);
    Token t1{}, t2{}, t3{};
    const auto h1 = t.create({.user = 1, .role = Role::Observer}, 100, t1);
    const auto h2 = t.create({.user = 2, .role = Role::Observer}, 200, t2);
    ASSERT_TRUE(t.touch(*h1, 300));  // h2 is now the least recently used
    const auto h3 = t.create({.user = 3, .role = Role::Admin}, 400, t3);
    EXPECT_EQ(h3->slot, h2->slot);
    EXPECT_FALSE(t.touch(*h2, 500));  // same slot, other generation: not the new session
    EXPECT_EQ(t.info(*h3)->user, 3);
    EXPECT_FALSE(t.find({t2.data(), t2.size()}, 500));
    EXPECT_TRUE(t.find({t1.data(), t1.size()}, 500));
}

TEST(Session, LogoutAndRemoveUser) {
    SessionTable<4> t(counting_random);
    Token a{}, b{}, c{};
    const auto ha = t.create({.user = 1, .role = Role::Admin}, 0, a);
    const auto hb = t.create({.user = 1, .role = Role::Admin}, 0, b);
    const auto hc = t.create({.user = 2, .role = Role::Observer}, 0, c);
    t.remove(*ha);
    EXPECT_FALSE(t.find({a.data(), a.size()}, 10));
    t.create({.user = 1, .role = Role::Admin}, 0, a);
    EXPECT_EQ(t.remove_user(1, *hb), 1u);  // the other session of user 1 goes, hb stays
    EXPECT_TRUE(t.alive(*hb, 10));
    EXPECT_TRUE(t.alive(*hc, 10));
}

TEST(Session, CookieParsingAndBase64url) {
    EXPECT_EQ(cookie_value("a=1; rtoos_session=abc; b=2", "rtoos_session"), "abc");
    EXPECT_EQ(cookie_value("rtoos_session=xyz", "rtoos_session"), "xyz");
    EXPECT_FALSE(cookie_value("rtoos_sessionx=1; x_rtoos_session=2", "rtoos_session"));
    EXPECT_FALSE(cookie_value("", "rtoos_session"));
    std::array<char, 4> out{};
    const std::array<std::byte, 3> in{std::byte{0xFB}, std::byte{0xFF}, std::byte{0xBF}};
    base64url(in, out);
    EXPECT_EQ(std::string_view(out.data(), 4), "-_-_");
}

// ---- throttle --------------------------------------------------------------------------------

TEST(Throttle, ExponentialDelayThenLockoutPerAddress) {
    LoginThrottle<4> th({.free_failures = 2, .base_delay_ms = 1000, .max_delay_ms = 8000, .lockout_failures = 7,
                         .lockout_ms = 60'000, .global_failures = 100});
    const std::uint32_t ip = 0x0A000001;
    std::uint32_t now = 0;
    th.failure(ip, false, now);
    th.failure(ip, false, now);
    EXPECT_EQ(th.wait_ms(ip, false, now), 0u);  // two free failures
    const std::uint32_t expected[] = {1000, 2000, 4000, 8000};
    for (std::uint32_t e : expected) {
        th.failure(ip, false, now);
        EXPECT_EQ(th.wait_ms(ip, false, now), e);
        now += e;
        EXPECT_EQ(th.wait_ms(ip, false, now), 0u);
    }
    th.failure(ip, false, now);  // 7th: lockout
    EXPECT_EQ(th.wait_ms(ip, false, now), 60'000u);
    EXPECT_EQ(th.wait_ms(0x0A000002, false, now), 0u);  // other addresses are free
    th.success(ip);
    EXPECT_EQ(th.wait_ms(ip, false, now), 0u);
    EXPECT_EQ(th.failures(ip), 0u);
}

TEST(Throttle, GlobalLockSparesTheLocalInterface) {
    LoginThrottle<8> th({.free_failures = 100, .lockout_failures = 200, .global_failures = 5, .global_window_ms = 10'000,
                         .global_lockout_ms = 30'000});
    for (std::uint32_t i = 0; i < 6; ++i) th.failure(0xC0A80000 + i, false, 1000 + i);
    EXPECT_GT(th.wait_ms(0xC0A800FF, false, 1010), 29'000u);  // everyone remote waits
    EXPECT_EQ(th.wait_ms(0xC0A8FF02, true, 1010), 0u);        // USB still works
    EXPECT_EQ(th.wait_ms(0xC0A800FF, false, 31'010), 0u);
    EXPECT_EQ(th.global_lockouts(), 1u);
}

TEST(Throttle, ManyAddressesReplaceTheOldestEntry) {
    LoginThrottle<2> th({.free_failures = 0, .base_delay_ms = 1000, .global_failures = 1000});
    th.failure(1, false, 0);
    th.failure(2, false, 10);
    th.failure(3, false, 20);  // replaces address 1
    EXPECT_EQ(th.failures(1), 0u);
    EXPECT_EQ(th.failures(2), 1u);
    EXPECT_EQ(th.failures(3), 1u);
}

// ---- roles and presence ----------------------------------------------------------------------

TEST(Access, RolesArePerPermission) {
    using enum Decision;
    EXPECT_EQ(decide({}, permission::read), NotAuthenticated);
    EXPECT_EQ(decide({.role = Role::Observer}, permission::read), Allowed);
    EXPECT_EQ(decide({.role = Role::Observer}, permission::operate), RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Operator}, permission::operate), Allowed);
    EXPECT_EQ(decide({.role = Role::Operator}, permission::program), RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Programmer}, permission::program), Allowed);
    EXPECT_EQ(decide({.role = Role::Programmer}, permission::admin), RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Admin}, permission::admin), Allowed);
    // a one-time password allows nothing until it is changed - not even reading
    EXPECT_EQ(decide({.role = Role::Admin, .must_change_password = true, .present = true}, permission::read),
              PasswordChangeRequired);
}

TEST(Access, CriticalActionsNeedPresence) {
    using enum Decision;
    EXPECT_EQ(decide({.role = Role::Programmer}, permission::program_critical), PresenceRequired);
    EXPECT_EQ(decide({.role = Role::Programmer, .present = true}, permission::program_critical), Allowed);
    EXPECT_EQ(decide({.role = Role::Operator, .present = true}, permission::program_critical), RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Admin}, permission::admin_critical), PresenceRequired);
    PresenceButton b(1000);
    EXPECT_FALSE(b.present(false, 0));
    EXPECT_TRUE(b.present(true, 0));  // USB
    b.pressed(5000);
    EXPECT_TRUE(b.present(false, 5999));
    EXPECT_EQ(b.remaining_ms(5400), 600u);
    EXPECT_FALSE(b.present(false, 6000));
    b.pressed(7000);
    b.consume();
    EXPECT_FALSE(b.present(false, 7001));
}

TEST(Access, RoleNames) {
    for (Role r : {Role::Observer, Role::Operator, Role::Programmer, Role::Admin}) EXPECT_EQ(parse_role(role_name(r)), r);
    EXPECT_FALSE(parse_role("root"));
}

// ---- credential store ------------------------------------------------------------------------

template <std::uint32_t BlockBytes, std::uint32_t Blocks, std::uint32_t ProgramBytes>
struct Nor {
    static constexpr std::uint32_t block_bytes = BlockBytes;
    static constexpr std::uint32_t block_count = Blocks;
    static constexpr std::uint32_t program_bytes = ProgramBytes;
    std::vector<std::uint8_t> cells = std::vector<std::uint8_t>(std::size_t{Blocks} * BlockBytes, 0xFF);
    long ops_left = -1;
    bool power() noexcept {
        if (ops_left == 0) return false;
        if (ops_left > 0) --ops_left;
        return true;
    }
    bool read(std::uint32_t block, std::uint32_t offset, std::span<std::byte> out) noexcept {
        if (block >= Blocks || offset + out.size() > BlockBytes) return false;
        std::memcpy(out.data(), &cells[std::size_t{block} * BlockBytes + offset], out.size());
        return true;
    }
    bool program(std::uint32_t block, std::uint32_t offset, std::span<const std::byte> data) noexcept {
        if (block >= Blocks || offset % ProgramBytes || data.size() % ProgramBytes || offset + data.size() > BlockBytes) return false;
        for (std::size_t q = 0; q < data.size(); q += ProgramBytes) {
            if (!power()) return false;
            std::memcpy(&cells[std::size_t{block} * BlockBytes + offset + q], data.data() + q, ProgramBytes);
        }
        return true;
    }
    bool erase(std::uint32_t block) noexcept {
        if (block >= Blocks || !power()) return false;
        std::fill_n(&cells[std::size_t{block} * BlockBytes], BlockBytes, std::uint8_t{0xFF});
        return true;
    }
};

UserRecord user(std::string_view name, Role role, std::uint8_t fill) {
    UserRecord u{};
    std::copy(name.begin(), name.end(), u.name.begin());
    u.role = role;
    u.password.iterations = 1000;
    u.password.hash.fill(std::byte{fill});
    return u;
}

TEST(CredentialStore, EmptyFlashThenSaveAndReload) {
    using Flash = Nor<8192, 2, 16>;
    Flash flash;
    CredentialStore<Flash, 8> store(flash);
    UserTable<8> t{};
    EXPECT_FALSE(store.load(t));
    t.users[0] = user("admin", Role::Admin, 1);
    t.users[3] = user("op", Role::Operator, 2);
    ASSERT_TRUE(store.save(t));
    t.users[3].role = Role::Observer;
    ASSERT_TRUE(store.save(t));
    CredentialStore<Flash, 8> again(flash);
    UserTable<8> loaded{};
    ASSERT_TRUE(again.load(loaded));
    EXPECT_EQ(again.sequence(), 2u);
    EXPECT_EQ(loaded.find("op"), std::optional<std::uint8_t>(3));
    EXPECT_EQ(loaded.users[3].role, Role::Observer);
    EXPECT_EQ(loaded.admins(), 1u);
    EXPECT_FALSE(loaded.find("nobody"));
}

TEST(CredentialStore, PowerLossAtEveryStepKeepsOldOrNewTable) {
    using Flash = Nor<131072, 2, 32>;  // STM32H7: 128 KB sectors, 32-byte program unit
    for (long cut = 0; cut < 40; ++cut) {
        Flash flash;
        UserTable<4> old_t{}, new_t{};
        old_t.users[0] = user("admin", Role::Admin, 1);
        {
            CredentialStore<Flash, 4> s(flash);
            UserTable<4> dummy{};
            (void)s.load(dummy);
            ASSERT_TRUE(s.save(old_t));
            new_t = old_t;
            new_t.users[0].password.hash.fill(std::byte{9});
            flash.ops_left = cut;
            (void)s.save(new_t);
            flash.ops_left = -1;
        }
        CredentialStore<Flash, 4> s(flash);
        UserTable<4> t{};
        ASSERT_TRUE(s.load(t)) << cut;
        const auto h = t.users[0].password.hash[0];
        EXPECT_TRUE(h == std::byte{1} || h == std::byte{9}) << cut;
    }
}

}  // namespace
