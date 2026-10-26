
from dotenv import load_dotenv
import json
import os
import sys
from pathlib import Path

from google import genai
from orbit_mcp_server import _Client


ROOT = Path(__file__).resolve().parent

# Try the newer model first, then fall back to the older model.
MODEL_CANDIDATES = [
    "gemini-3.8-flash",
    "gemini-3.6-flash",
]

READ_ONLY_TOOLS = [
    "get_pose",
    "get_status",
    "get_capabilities",
    "house_info",
]


def generate_summary(gemini, contents):
    """Generate a summary, falling back if a model is unavailable."""
    last_error = None

    for index, model in enumerate(MODEL_CANDIDATES):
        print(f"\nTrying Gemini model: {model}")

        try:
            response = gemini.models.generate_content(
                model=model,
                contents=contents,
            )

            if not response.text:
                raise RuntimeError(
                    f"{model} returned an empty response."
                )

            print(f"Selected model: {model}")
            return response.text

        except Exception as exc:
            last_error = exc
            error_text = str(exc).lower()

            model_unavailable = any(
                phrase in error_text
                for phrase in (
                    "model not found",
                    "not supported for generatecontent",
                    "is not found",
                    "unsupported model",
                    "unknown model",
                )
            )

            # Fall back only for model availability errors.
            if (
                model_unavailable
                and index < len(MODEL_CANDIDATES) - 1
            ):
                print(
                    f"{model} is unavailable. "
                    "Trying the fallback model."
                )
                continue

            raise

    raise RuntimeError(
        f"All configured models failed. Last error: {last_error}"
    )


def main():
    # Load the API key from the .env file in this project folder.
    load_dotenv(ROOT / ".env", override=True)

    api_key = os.environ.get("GEMINI_API_KEY", "").strip()

    if not api_key:
        raise RuntimeError(
            "GEMINI_API_KEY is missing from the project's .env file."
        )

    # Start ORBIT MCP as a separate process.
    command = [
        sys.executable,
        str(ROOT / "orbit_mcp_server.py"),
        "--robot", "nao",
        "--sim", "webots",
    ]

    client = _Client(command)

    try:
        # Initialize the MCP connection.
        init = client.request(
            "initialize",
            {
                "protocolVersion": "2025-06-18",
                "capabilities": {},
                "clientInfo": {
                    "name": "gemini-readonly-client",
                    "version": "1.0",
                },
            },
        )

        if "error" in init:
            raise RuntimeError(
                f"MCP initialization failed: {init['error']}"
            )

        client.notify("notifications/initialized")

        # Discover the tools exposed by the MCP server.
        listing = client.request("tools/list")

        if "error" in listing:
            raise RuntimeError(
                f"MCP tools/list failed: {listing['error']}"
            )

        tools = listing["result"]["tools"]
        available = {tool["name"] for tool in tools}

        print("\n=== MCP CONNECTION ===")
        print("Connected to ORBIT MCP server.")
        print(f"Discovered tools: {len(tools)}")

        results = {}

        # Call only read-only telemetry tools.
        for name in READ_ONLY_TOOLS:
            if name not in available:
                results[name] = {
                    "status": "UNAVAILABLE",
                    "detail": "Tool was not listed by the MCP server.",
                }
                print(f"{name}: UNAVAILABLE")
                continue

            try:
                reply = client.request(
                    "tools/call",
                    {
                        "name": name,
                        "arguments": {},
                    },
                )

                if "error" in reply:
                    results[name] = {
                        "status": "ERROR",
                        "detail": reply["error"],
                    }
                else:
                    result = reply["result"]

                    results[name] = {
                        "status": (
                            "ERROR"
                            if result.get("isError", False)
                            else "OK"
                        ),
                        "result": result,
                    }

                print(f"\n=== {name} ===")
                print(
                    json.dumps(
                        results[name],
                        indent=2,
                        ensure_ascii=False,
                    )
                )

            except Exception as exc:
                results[name] = {
                    "status": "ERROR",
                    "detail": str(exc),
                }
                print(f"{name}: ERROR: {exc}")

        # Send the read-only results to Gemini for summarization.
        print("\n=== GEMINI SUMMARY ===")

        gemini = genai.Client(api_key=api_key)

        summary_prompt = (
            "You are summarizing read-only telemetry from an "
            "ORBIT ROS NAO robot MCP server. Use only the supplied "
            "results. Do not infer missing values or claim physical "
            "movement. Clearly identify unavailable tools and errors. "
            "Do not issue commands. Summarize in concise English.\n\n"
            + json.dumps(
                results,
                ensure_ascii=False,
                default=str,
            )
        )

        summary = generate_summary(gemini, summary_prompt)
        print("\n" + summary)

    finally:
        client.close(wait=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(
            f"\nFAILED: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        sys.exit(1)
