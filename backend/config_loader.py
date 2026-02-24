import json
import os
import sys


def load_config() -> dict:
    """Load config from project root config.json."""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config.json')
    config_path = os.path.normpath(config_path)

    try:
        with open(config_path) as f:
            config = json.load(f)
    except FileNotFoundError:
        print(f"ERROR: Config file not found at {config_path}", file=sys.stderr)
        print("Ensure config.json exists in the project root.", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON in {config_path}: {e}", file=sys.stderr)
        sys.exit(1)

    # Validate required keys
    required = [
        ('aws', 'region'),
        ('iot', 'deviceId'),
        ('iot', 'topicPrefix'),
    ]
    for keys in required:
        obj = config
        for key in keys:
            if key not in obj:
                print(
                    f"ERROR: Missing config key: {'.'.join(keys)}", file=sys.stderr
                )
                sys.exit(1)
            obj = obj[key]

    return config
