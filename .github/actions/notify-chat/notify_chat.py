# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sends a Cards V2 build status notification to Google Chat via REST API."""

import base64
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
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
        return subprocess.check_output(
            ['git', 'log', '-1', f'--format={fmt}'],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return ''


def main():
    space_id = os.environ.get('CHAT_SPACE_ID', 'spaces/AAAAzcCLt1A')
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

    message_id = f'client-run-{run_id}'
    url = f'https://chat.googleapis.com/v1/{space_id}/messages?messageId={message_id}'
    post_req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode('utf-8'),
        headers={
            'Authorization': f'Bearer {token}',
            'Content-Type': 'application/json; charset=UTF-8',
        },
        method='POST',
    )

    try:
        with urllib.request.urlopen(post_req) as response:
            resp_data = json.loads(response.read().decode('utf-8'))
            print('Chat notification sent successfully!')
    except urllib.error.HTTPError as e:
        if e.code == 409:
            print(f'Notification for run {run_id} was already posted by another job; skipping duplicate.')
            sys.exit(0)
        print(f'Failed to post chat message (HTTP {e.code}): {e.read().decode("utf-8")}', file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'Failed to post chat message: {e}', file=sys.stderr)
        sys.exit(1)

    user_id_map_str = os.environ.get('CHAT_USER_ID_MAP', '')
    chat_user_id_map = {}
    if user_id_map_str:
        try:
            chat_user_id_map = json.loads(user_id_map_str)
        except Exception as e:
            print(f'Failed to parse CHAT_USER_ID_MAP: {e}', file=sys.stderr)

    email = email.strip().lower() if email else ''
    chat_user_id = chat_user_id_map.get(email)
    thread_name = resp_data.get('thread', {}).get('name')
    if chat_user_id:
        if thread_name:
            reply_payload = {
                'text': f'cc <users/{chat_user_id}>',
                'thread': {'name': thread_name},
            }
            reply_url = f'https://chat.googleapis.com/v1/{space_id}/messages?messageReplyOption=REPLY_MESSAGE_FALLBACK_TO_NEW_THREAD&messageId={message_id}-cc'
            reply_req = urllib.request.Request(
                reply_url,
                data=json.dumps(reply_payload).encode('utf-8'),
                headers={
                    'Authorization': f'Bearer {token}',
                    'Content-Type': 'application/json; charset=UTF-8',
                },
                method='POST',
            )
            try:
                with urllib.request.urlopen(reply_req) as reply_resp:
                    print('Threaded mention reply sent successfully!')
            except urllib.error.HTTPError as e:
                if e.code == 409:
                    print(f'Threaded mention for run {run_id} was already posted; skipping duplicate.')
                else:
                    print(f'Failed to post threaded mention (HTTP {e.code}): {e.read().decode("utf-8")}', file=sys.stderr)
            except Exception as e:
                print(f'Failed to post threaded mention: {e}', file=sys.stderr)
        else:
            print(f'Warning: User ID found for {email}, but no thread name in chat response. Skipping threaded mention.', file=sys.stderr)


if __name__ == '__main__':
    main()
