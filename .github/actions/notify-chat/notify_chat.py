"""Sends a Cards V2 build status notification to Google Chat via REST API."""

import base64
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request


def get_service_account_token(key_data: dict, scope: str = 'https://www.googleapis.com/auth/chat.messages.create') -> str:
    header = {'alg': 'RS256', 'typ': 'JWT'}
    now = int(time.time())
    payload = {
        'iss': key_data['client_email'],
        'scope': scope,
        'aud': key_data.get('token_uri', 'https://oauth2.googleapis.com/token'),
        'exp': now + 3600,
        'iat': now,
    }

    def b64url(data: bytes) -> str:
        return base64.urlsafe_b64encode(data).decode('utf-8').rstrip('=')

    unsigned_jwt = f"{b64url(json.dumps(header).encode('utf-8'))}.{b64url(json.dumps(payload).encode('utf-8'))}"

    with tempfile.NamedTemporaryFile('w', delete=False) as f:
        f.write(key_data['private_key'])
        key_file = f.name
    try:
        proc = subprocess.Popen(
            ['openssl', 'dgst', '-sha256', '-sign', key_file],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        sig, err = proc.communicate(unsigned_jwt.encode('utf-8'))
        if proc.returncode != 0:
            raise RuntimeError(f"OpenSSL signing failed: {err.decode('utf-8')}")
        signed_jwt = f"{unsigned_jwt}.{b64url(sig)}"
    finally:
        if os.path.exists(key_file):
            os.remove(key_file)

    token_url = key_data.get('token_uri', 'https://oauth2.googleapis.com/token')
    data = urllib.parse.urlencode({
        'grant_type': 'urn:ietf:params:oauth:grant-type:jwt-bearer',
        'assertion': signed_jwt,
    }).encode('utf-8')
    req = urllib.request.Request(token_url, data=data)
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read().decode('utf-8'))['access_token']


def git_log(fmt: str) -> str:
    try:
        return subprocess.check_output(['git', 'log', '-1', f'--format={fmt}'], text=True).strip()
    except Exception:
        return ''


def main():
    space_id = os.environ.get('CHAT_SPACE_ID', 'spaces/AAQAJJIPgX8')
    if not space_id.startswith('spaces/'):
        space_id = f'spaces/{space_id}'

    token = os.environ.get('CHAT_ACCESS_TOKEN')
    sa_key_str = os.environ.get('CHAT_SERVICE_ACCOUNT_KEY')

    if not token and sa_key_str:
        if sa_key_str.strip().startswith('{'):
            key_data = json.loads(sa_key_str)
        else:
            with open(sa_key_str, 'r', encoding='utf-8') as f:
                key_data = json.load(f)
        token = get_service_account_token(key_data)

    if not token:
        print('Skipping chat notification (no credentials provided).')
        sys.exit(0)

    server_url = os.environ.get('GITHUB_SERVER_URL', 'https://github.com')
    repo = os.environ.get('GITHUB_REPOSITORY', 'google-deepmind/mujoco')
    run_id = os.environ.get('GITHUB_RUN_ID', '123456')
    run_url = os.environ.get('JOB_URL', f'{server_url}/{repo}/actions/runs/{run_id}')

    commit_sha = os.environ.get('CHAT_COMMIT_SHA', '')[:7] or os.environ.get('GITHUB_SHA', '')[:7] or git_log('%h') or 'unknown'
    git_name = git_log('%an')
    git_email = git_log('%ae')
    git_msg = git_log('%s')

    author = os.environ.get('CHATMSG_AUTHOR_NAME') or git_name or os.environ.get('GITHUB_ACTOR', 'Unknown Author')
    email = os.environ.get('CHATMSG_AUTHOR_EMAIL') or git_email
    commit_msg = os.environ.get('CHATMSG_COMMIT_MESSAGE') or git_msg or 'Build failed'

    if author == email and git_name and git_email:
        author = git_name
        email = git_email

    if author and email and author != email:
        author_display = f'{author} <{email}>'
    elif author:
        author_display = author
    else:
        author_display = 'Unknown Author'

    payload = {
        'cardsV2': [
            {
                'cardId': 'buildStatusCard',
                'card': {
                    'header': {
                        'title': 'Build Status: FAILURE',
                        'subtitle': f"Workflow: {os.environ.get('GITHUB_WORKFLOW', 'build')}",
                        'imageUrl': 'https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png',
                        'imageType': 'CIRCLE',
                    },
                    'sections': [
                        {
                            'widgets': [
                                {
                                    'decoratedText': {
                                        'text': f'🔴 Run: <a href="{run_url}">{run_id}</a>',
                                    }
                                }
                            ]
                        },
                        {
                            'widgets': [
                                {
                                    'textParagraph': {
                                        'text': f'```\nCommit: {commit_sha}\nAuthor: {author_display}\n\n{commit_msg}\n```',
                                        'textSyntax': 'MARKDOWN',
                                    }
                                }
                            ]
                        },
                    ],
                },
            }
        ]
    }

    url = f'https://chat.googleapis.com/v1/{space_id}/messages'
    post_req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode('utf-8'),
        headers={
            'Authorization': f'Bearer {token}',
            'Content-Type': 'application/json',
        },
        method='POST',
    )

    try:
        with urllib.request.urlopen(post_req) as response:
            print('Chat notification sent successfully!')
    except Exception as e:
        print(f'Failed to post chat message: {e}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
