try:
    import hailo_sdk_client;
    from hailo_sdk_client import ClientRunner, InferenceContext

    print(f"Hailo SDK found, client version is: {hailo_sdk_client.__version__}")
except ImportError:
    print(
        "ERROR: hailo_sdk_client not found.\n"
        "\n"
        "The Hailo Dataflow Compiler is proprietary and not available on PyPI.\n"
        "Steps to obtain it:\n"
        "  1. Register at https://hailo.ai/developer-zone/ (free account)\n"
        "  2. Download 'Hailo Dataflow Compiler' (hailo_dataflow_compiler-*.whl)\n"
        "     and 'hailo_sdk_client-*.whl' from the Software Downloads section\n"
        "  3. Install both wheels in your environment:\n"
        "       pip install hailo_dataflow_compiler-*.whl\n"
        "       pip install hailo_sdk_client-*.whl\n"
        "  4. Re-run this script\n",
        file=sys.stderr,
    )
