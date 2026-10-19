#!/usr/bin/env python3
"""构建辅助：生成源码清单和C++头，运行健康判断不依赖Python。

输入：本包源码/接口/配置/启动文件。
输出：source_build_manifest.json及source_build_manifest.h，编进本次核心库。
同名输出内容没有变化时不重写，避免无意义的全量重新编译。
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def digest(path):
    """只读计算单个源码材料摘要；这里不对输入bag进行哈希。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_value(repo, arguments):
    """可选记录相邻FAST-LIO仓库状态；独立部署没有该仓库也可以正常构建。"""
    if not repo.exists():
        return 'unavailable'
    result = subprocess.run(['git', '-C', str(repo)] + arguments,
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else 'unavailable'


def main():
    """枚举材料 -> 求摘要 -> 可选Git上下文 -> 保存JSON及可被C++包含的常量。"""
    root, output = map(Path, sys.argv[1:3])
    files = {}
    for folder in ('src', 'include', 'msg', 'srv', 'launch', 'config', 'test', 'scripts'):
        for path in sorted((root / folder).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                files[str(path.relative_to(root))] = digest(path)
    data = {'source_files_sha256': files}
    repo = root.parent / 'FAST_LIO'
    if repo.exists():
        data['fastlio_commit'] = git_value(repo, ['rev-parse', 'HEAD'])
        data['fastlio_dirty'] = git_value(repo, ['status', '--porcelain'])
    payload = json.dumps(data, ensure_ascii=False, indent=2) + '\n'
    header = '#pragma once\n// 本次编译材料快照，不是运行时健康规则。\n'
    header += 'static const char* const robot_health_source_manifest = '
    header += json.dumps(payload, ensure_ascii=True) + ';\n'
    for path, text in ((output, payload), (output.with_suffix('.h'), header)):
        if not path.exists() or path.read_text() != text:
            path.write_text(text)


if __name__ == '__main__':
    main()
