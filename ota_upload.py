
import os

Import("env")

# Локальний файл із паролем, не для GitHub
password_file = os.path.join(
    env["PROJECT_DIR"], "ota_password.txt"
)

if os.path.isfile(password_file):
    with open(password_file, "r", encoding="utf-8") as f:
        password = f.read().strip()

    if password:
        env.Append(
            UPLOAD_FLAGS=[
                "--auth=" + password,
                "--host_port=29040"
            ]
        )
