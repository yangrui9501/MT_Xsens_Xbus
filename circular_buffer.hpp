/**
 * @file circular_buffer.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <Arduino.h>

#define BUFFER_SIZE 512

// 溢位處理策略
enum OverflowPolicy {
    REJECT_NEW,    // 拒絕新資料（原本的行為）
    OVERWRITE_OLD  // 覆寫舊資料
};

class ByteBuffer {
private:
    uint8_t buffer[BUFFER_SIZE];
    volatile int head;
    volatile int tail;
    volatile int count;
    OverflowPolicy policy;  // 新增：溢位處理策略

public:
    // 建構函式可指定策略
    ByteBuffer(OverflowPolicy p = OVERWRITE_OLD) {
        head = 0;
        tail = 0;
        count = 0;
        policy = p;
    }

    // 修改後的 write 函式
    bool write(uint8_t data) {

        if (count >= BUFFER_SIZE) {
            if (policy == REJECT_NEW) {
                return false;  // 原本的行為：拒絕寫入
            }
            else {
                // OVERWRITE_OLD：強制覆寫
                // 當緩衝區滿時，tail 也要跟著前進
                tail = (tail + 1) % BUFFER_SIZE;
                count--;  // 因為會覆寫一個舊資料
            }
        }

        buffer[head] = data;
        head = (head + 1) % BUFFER_SIZE;
        count++;

        return true;
    }

    // 讀取函式不變
    uint8_t read() {
        if (count == 0) {
            return 0x00;
        }

        uint8_t data = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;

        return data;
    }
    bool read(uint8_t* data) {
        if (count == 0) {
            return false;
        }

        *data = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;

        return true;
    }

    // 設定溢位策略
    void setOverflowPolicy(OverflowPolicy p) {
        policy = p;
    }

    OverflowPolicy getOverflowPolicy() {
        return policy;
    }

    // 其他函式保持不變
    int available() { return count; }
    bool isEmpty() { return count == 0; }
    bool isFull() { return count >= BUFFER_SIZE; }

    void clear() {
        head = 0;
        tail = 0;
        count = 0;

    }

    // 除錯用：印出狀態
    void printStatus() {
        Serial.println("=== Buffer 狀態 ===");
        Serial.print("策略: ");
        Serial.println(policy == REJECT_NEW ? "拒絕新資料" : "覆寫舊資料");

        Serial.print("Buffer 內容: [");
        for (int i = 0; i < BUFFER_SIZE; i++) {
            Serial.print("0x");
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
            if (i < BUFFER_SIZE - 1) Serial.print(", ");
        }
        Serial.println("]");

        Serial.print("索引標示:    [");
        for (int i = 0; i < BUFFER_SIZE; i++) {
            if (i == head && i == tail) {
                Serial.print("HT");
            }
            else if (i == head) {
                Serial.print("H ");
            }
            else if (i == tail) {
                Serial.print("T ");
            }
            else {
                Serial.print("  ");
            }
            if (i < BUFFER_SIZE - 1) Serial.print(", ");
        }
        Serial.println("]");

        Serial.print("head=");
        Serial.print(head);
        Serial.print(", tail=");
        Serial.print(tail);
        Serial.print(", count=");
        Serial.println(count);
        Serial.println();
    }
};