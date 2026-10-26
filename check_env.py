"""Show which API providers are configured, without printing any key.   python check_env.py
Exit code 0 if at least one provider in LLM_PROVIDER_ORDER has a key, 1 otherwise (the Oracle/Webots runs need no key at all)."""
import os, sys
from orbit_ros.env_config import load_env, format_status, provider_status, project_root

def main():
    path = os.path.join(project_root(), ".env")
    if not os.path.isfile(path): print(f"No .env file at {path}\nCreate it with:  Copy-Item .env.example .env   (then paste your keys)")
    else: load_env(path); print(f".env found at {path}")
    print(format_status())
    ok = any(r["configured"] and r["in_order"] for r in provider_status())
    print("\nAt least one provider is configured." if ok else "\nNo provider key configured. Webots and Oracle runs still work; live LLM runs need a key.")
    return 0 if ok else 1

if __name__ == "__main__": sys.exit(main())
