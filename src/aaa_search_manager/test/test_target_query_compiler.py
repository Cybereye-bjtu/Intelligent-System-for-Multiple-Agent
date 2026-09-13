from aaa_search_manager.query_compiler_contract import compiler_command


def test_compiler_command_is_argument_safe_and_local():
    command = compiler_command(
        "/runner.sh",
        text='Find the sign labeled "BANK".',
        model="/models/qwen",
        adapter="/models/lora",
        mode="hybrid",
        max_new_tokens=128,
    )
    assert command[:3] == ["/runner.sh", "-m", "cybereye_query_parser.cli"]
    assert command[command.index("--text") + 1] == 'Find the sign labeled "BANK".'
    assert command[command.index("--model") + 1] == "/models/qwen"
    assert command[command.index("--adapter") + 1] == "/models/lora"
