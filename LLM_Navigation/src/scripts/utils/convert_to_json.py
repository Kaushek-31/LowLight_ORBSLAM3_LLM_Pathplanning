#!/usr/bin/env python3
import ast
import json
import sys
import os

def extract_function_text(filename):
    """
    Extract function text from a Python file and return it in a dictionary format.

    Parameters:
    - filename (str): The path to the Python file.

    Returns:
    - dict: A dictionary where keys are function names and values are the text of the function.
    """

    if not os.path.isfile(filename):
        raise FileNotFoundError(f"File not found: {filename}")

    with open(filename, 'r') as f:
        source = f.read()
        node = ast.parse(source)

    functions = [n for n in node.body if isinstance(n, ast.FunctionDef)]
    function_texts = {}

    lines = source.splitlines(keepends=True)

    for function in functions:
        start_line = function.lineno - 1
        end_line = function.end_lineno if hasattr(function, 'end_lineno') else start_line
        func_text = ''.join(lines[start_line:end_line + 1])
        function_texts[function.name] = func_text

    return function_texts

if __name__ == '__main__':
    # For testing purposes from CLI
    if len(sys.argv) != 2:
        print("Usage: python convert_to_json.py <python_file>")
        sys.exit(1)

    filename = sys.argv[1]
    try:
        functions = extract_function_text(filename)
        print(json.dumps(functions, indent=2))
    except Exception as e:
        print(f"Error: {e}")
