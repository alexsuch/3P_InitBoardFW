// mavlink_crc_helper.h
// Minimal MAVLink v2 CRC helper (X.25), без залежностей і таблиць.
// ПРИЗНАЧЕННЯ: порахувати пакетний CRC = CRC(header без magic) + CRC(payload) + CRC(crc_extra).
//
// ──────────────────────────────────────────────────────────────────────────────
// ДЕ ВЗЯТИ crc_extra (ОФІЦІЙНО):
//  • Згенерувати C-бібліотеку з вашого MAVLink XML (dialect) через pymavlink/mavgen:
//      python -m pymavlink.tools.mavgen --lang C --wire-protocol 2.0 --output out <your>.xml
//    Потім узяти числа з згенерованих хедерів:
//      - у файлах виду mavlink_msg_<name>.h є макроси MAVLINK_MSG_ID_<NAME>_CRC
//      - або у зведеному масиві MAVLINK_MESSAGE_CRCS (залежить від генератора/версії).
//
// ЯК ЗГЕНЕРУВАТИ САМОСТІЙНО (БЕЗ ПІДТЯГУВАННЯ ВСІЄЇ БІБЛІОТЕКИ):
//  • Поза МК (офлайн) проганяєте ваш(і) XML тим самим mavgen і просто копіюєте потрібні
//    значення у маленький lookup (switch/масив) у прошивку.
//  • Примітка: crc_extra детерміновано обчислюється з “сигнатури” повідомлення
//    (назва та список полів у порядку on-wire, без extension-полів MAVLink2) тим же CRC-16/X.25.
//    Найнадійніше — довіритись офіційному генератору, щоб уникнути розбіжностей.
//
// ВАЖЛИВО ПРО РОЗРАХУНОК MAVLink v2:
//  • У CRC НЕ входить magic (STX=0xFD).
//  • Спочатку додаємо core-header БЕЗ magic (9 байт): len, incompat, compat, seq, sysid, compid, msgid[0..2].
//  • Додаємо payload рівно len байт (без підпису/чексума).
//  • Наприкінці додаємо один байт crc_extra для конкретного msgid.
//  • Підпис (signature, 13 байт) у CRC не входить.
// ──────────────────────────────────────────────────────────────────────────────

#ifndef MAVLINK_CRC_HELPER_H
#define MAVLINK_CRC_HELPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// X.25 accumulate (як у c_library_v2): без таблиць і зайвих залежностей.
static inline void mav_crc_accumulate(uint8_t data, uint16_t *crc) {
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

static inline void mav_crc_accumulate_buf(uint16_t *crc, const uint8_t *buf, uint16_t len) {
    while (len--) mav_crc_accumulate(*buf++, crc);
}

// Основний хелпер для MAVLink v2.
// header_no_magic  → вказівник на LEN (тобто packet+1),  header_len → 9 (ядро v2 без magic)
// payload          → вказівник на початок payload,        payload_len → packet[1] (LEN)
// crc_extra        → байт із офіційних заголовків/генератора для цього msgid
static inline uint16_t mavlink2_crc_x25(const uint8_t *header_no_magic, uint8_t header_len, const uint8_t *payload, uint8_t payload_len, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;  // X.25 init
    mav_crc_accumulate_buf(&crc, header_no_magic, (uint16_t)header_len);
    mav_crc_accumulate_buf(&crc, payload, (uint16_t)payload_len);
    mav_crc_accumulate(crc_extra, &crc);
    return crc;
}

// Дрібний довідник crc_extra для найуживаніших повідомлень.
// За потреби розширюйте лише тими ID, які реально використовуєте у вашому діалекті.
static inline uint8_t mav_crc_extra_lookup_minimal(uint32_t msgid) {
    switch (msgid) {
        case 0:
            return 50;  // HEARTBEAT
        case 30:
            return 39;  // ATTITUDE
        case 74:
            return 20;  // VFR_HUD
        case 76:
            return 152;  // COMMAND_LONG
        case 77:
            return 143;  // COMMAND_ACK
        // …додайте свої повідомлення за даними з офіційного генератора…
        default:
            return 0;  // невідомий msgid → свідомо провалити перевірку CRC
    }
}

// (Опціонально) Зручний обгортковий варіант "з цілого пакета" MAVLink v2.
// Очікує формат: [0]=0xFD, [1]=len, [2]=incompat, [3]=compat, [4]=seq,
// [5]=sysid, [6]=compid, [7..9]=msgid LSB→MSB, [10..]=payload, потім 2 байти CRC.
static inline uint16_t mavlink2_crc_from_packet(const uint8_t *packet) {
    const uint8_t len = packet[1];
    const uint8_t *hdr = &packet[1];  // без magic
    const uint8_t hdr_len = 9;        // v2 core header
    const uint8_t *pl = &packet[10];

    const uint32_t msgid = (uint32_t)packet[7] | ((uint32_t)packet[8] << 8) | ((uint32_t)packet[9] << 16);

    const uint8_t crc_extra = mav_crc_extra_lookup_minimal(msgid);
    return mavlink2_crc_x25(hdr, hdr_len, pl, len, crc_extra);
}

#ifdef __cplusplus
}
#endif
#endif  // MAVLINK_CRC_HELPER_H
