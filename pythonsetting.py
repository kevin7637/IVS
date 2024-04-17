import os
import subprocess

# 리포지토리 URL (사용자 이름 없이)
repo_url = "https://github.com/kevin7637/IVS.git"

# 직접 GitHub 토큰 할당
token = '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'

# 인증 토큰을 URL에 포함
auth_url = repo_url.replace("https://", f"https://{token}@")

# 'IVS' 폴더 삭제
subprocess.run(["rm", "-rf", "IVS"])

# 리포지토리 클론
subprocess.run(["git", "clone", auth_url], check=True)

print("리포지토리 클론이 완료되었습니다.")
