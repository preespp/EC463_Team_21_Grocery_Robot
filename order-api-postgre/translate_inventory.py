import json
import sys

from deep_translator import GoogleTranslator


def main():
    target = (sys.argv[1] if len(sys.argv) > 1 else "en").strip().lower()
    # map alias codes to supported deep_translator codes
    target_map = {
        "": "en",
        "en-us": "en",
        "es-es": "es",
        "es-mx": "es",
        "zh": "zh-CN",
        "zh-cn": "zh-CN",
        "zh-hans": "zh-CN"
    }
    target = target_map.get(target, target)

    raw = sys.stdin.read()
    payload = json.loads(raw) if raw else []
    texts = [str(x) for x in payload] if isinstance(payload, list) else [str(payload)]

    if target in ("", "en", "es", "zh-CN") and target == "en":
        print(json.dumps(texts, ensure_ascii=False))
        return

    # ensure we only call translator for supported non-en cases
    if target == "en":
        print(json.dumps(texts, ensure_ascii=False))
        return

    translator = GoogleTranslator(source="en", target=target)
    translated = []
    for text in texts:
        if not text:
            translated.append("")
            continue

        # retries for transient connection errors
        attempts = 0
        success = False
        while attempts < 3 and not success:
            attempts += 1
            try:
                translated.append(translator.translate(text))
                success = True
            except Exception as ex:
                if attempts >= 3:
                    # if still failing, keep original text as fallback
                    translated.append(text)
                else:
                    # small backoff; avoid endless immediate failure
                    import time
                    time.sleep(0.7)

    print(json.dumps(translated, ensure_ascii=False))


if __name__ == "__main__":
    main()
