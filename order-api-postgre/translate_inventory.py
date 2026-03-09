import json
import sys

from deep_translator import GoogleTranslator


def main():
    target = (sys.argv[1] if len(sys.argv) > 1 else "en").strip().lower()
    raw = sys.stdin.read()
    payload = json.loads(raw) if raw else []
    texts = [str(x) for x in payload] if isinstance(payload, list) else [str(payload)]

    if target in ("", "en", "en-us"):
        print(json.dumps(texts, ensure_ascii=False))
        return

    translator = GoogleTranslator(source="en", target=target)
    translated = []
    for text in texts:
        if not text:
            translated.append("")
            continue
        translated.append(translator.translate(text))

    print(json.dumps(translated, ensure_ascii=False))


if __name__ == "__main__":
    main()
