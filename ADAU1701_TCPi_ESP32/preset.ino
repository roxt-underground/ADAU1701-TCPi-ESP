#include <Arduino.h>

bool parseHexByte(const char* s, size_t start, size_t end, uint8_t& value) {
    char buf[8];
    const size_t n = end - start;
    if (n >= sizeof(buf)) return false;  // заведомо слишком длинное поле

    for (size_t i = 0; i < n; ++i) {
        buf[i] = s[start + i];
    }
    buf[n] = '\0';

    // поле не должно быть пустым (или только из пробелов)
    const char* p = buf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p == '\0') return false;

    char* endptr = nullptr;
    long v = strtol(buf, &endptr, 16);

    // после числа может идти только хвост из пробелов
    while (*endptr == ' ' || *endptr == '\t' || *endptr == '\r' || *endptr == '\n') ++endptr;
    if (*endptr != '\0') return false;   // мусор, например "12zz"

    if (v < 0 || v > 0xFF) return false; // не помещается в один байт

    value = (uint8_t)v;
    return true;
}


// Разбивает строку по запятым, парсит каждое значение и заполняет массив:
//   output[0] — количество байтов, output[1..n] — сами значения.
bool serializeBytes(const String input, uint8_t* output, size_t maxLen) {
    if (output == nullptr || maxLen < 2) {
        return false;
    }

    const char* p = input.c_str();
    const size_t len = input.length();

    // 1-й проход: считаем поля и проверяем каждое (разделитель — запятая)
    size_t count = 0;
    size_t start = 0;
    for (size_t i = 0; i <= len; ++i) {
        if (i < len && p[i] != ',') continue;
        uint8_t v;
        if (!parseHexByte(p, start, i, v)) return false;
        if (++count + 1 > maxLen) return false;
        start = i + 1;
    }

    // 2-й проход: заполняем массив (строка уже проверена)
    output[0] = (uint8_t)count;
    size_t idx = 1;
    start = 0;
    for (size_t i = 0; i <= len; ++i) {
        if (i < len && p[i] != ',') continue;
        uint8_t v;
        if (!parseHexByte(p, start, i, v)) return false;
        output[idx++] = v;
        start = i + 1;
    }
    return true;
}