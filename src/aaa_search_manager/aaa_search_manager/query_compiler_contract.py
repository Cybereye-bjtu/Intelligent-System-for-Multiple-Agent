"""Pure helpers shared by the ROS natural-language compiler node and tests."""


def compiler_command(runner, *, text, model, adapter, mode, max_new_tokens):
    return [
        str(runner), "-m", "cybereye_query_parser.cli",
        "--text", str(text), "--model", str(model), "--adapter", str(adapter),
        "--mode", str(mode), "--output", "query",
        "--max-new-tokens", str(int(max_new_tokens)), "--device-map", "auto",
    ]
