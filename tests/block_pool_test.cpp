// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <atomic>
#include <set>
#include <thread>
#include <vector>

#include "rtos/kernel/block_pool.hpp"

namespace {

TEST(BlockPool, HandsOutEveryBlockExactlyOnceThenReportsExhaustion) {
    rtos::BlockPool<24, 4, 8> pool;
    static_assert(decltype(pool)::stride == 24);
    std::set<void*> blocks;
    for (int i = 0; i < 4; ++i) {
        void* p = pool.allocate();
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(reinterpret_cast<std::uintptr_t>(p) % 8, 0u);
        EXPECT_TRUE(pool.owns(p));
        blocks.insert(p);
    }
    EXPECT_EQ(blocks.size(), 4u);
    EXPECT_EQ(pool.allocate(), nullptr);
    EXPECT_EQ(pool.available(), 0u);

    pool.deallocate(*blocks.begin());
    EXPECT_EQ(pool.available(), 1u);
    EXPECT_EQ(pool.allocate(), *blocks.begin());  // LIFO: hot block reused
}

TEST(BlockPool, StrideRoundsUpToAlignment) {
    static_assert(rtos::BlockPool<10, 2, 32>::stride == 32);
}

TEST(BlockPoolDeathTest, ForeignPointerViolatesContract) {
    rtos::BlockPool<16, 2> pool;
    int foreign = 0;
    EXPECT_DEATH(pool.deallocate(&foreign), "assertion_kind: assert");
}

TEST(BlockPool, ConcurrentAllocateFreeKeepsBlocksExclusive) {
    constexpr std::size_t blocks = 64;
    rtos::BlockPool<32, blocks> pool;
    std::array<std::atomic<int>, blocks> owners{};
    std::atomic<bool> corrupted{false};

    auto worker = [&] {
        for (int i = 0; i < 50000; ++i) {
            void* p = pool.allocate();
            if (p == nullptr) {
                continue;
            }
            // Blocks are contiguous with stride 32 -> address/32 mod N is unique per block.
            auto& owner = owners[(reinterpret_cast<std::uintptr_t>(p) / 32) % blocks];
            if (owner.fetch_add(1) != 0) {
                corrupted = true;  // two threads hold the same block
            }
            owner.fetch_sub(1);
            pool.deallocate(p);
        }
    };
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back(worker);
    }
    for (auto& t : threads) {
        t.join();
    }
    EXPECT_FALSE(corrupted.load());
    EXPECT_EQ(pool.available(), blocks);
}

}  // namespace
