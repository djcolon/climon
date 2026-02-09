import subprocess
import os

# Get the short hash of the current commit
result = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], 
                       capture_output=True, text=True, check=True)
short_hash = result.stdout.strip()

# Write to src/version.h
os.makedirs('src', exist_ok=True)
with open('src/version.h', 'w') as f:
    f.write(f'#define VERSION "{short_hash}"\n')

print(f"Written commit hash: {short_hash}")