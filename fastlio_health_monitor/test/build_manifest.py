#!/usr/bin/env python3
"""Build metadata only; runtime functionality lives in C++."""
import hashlib,json,subprocess,sys
from pathlib import Path
root,out=map(Path,sys.argv[1:3])
data={'source_files_sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ('src','include','msg','srv') for p in sorted((root/folder).rglob('*')) if p.is_file()}}
repo=root.parent/'FAST_LIO'
for name,args in (('fastlio_commit',['rev-parse','HEAD']),('fastlio_dirty',['status','--porcelain'])):
    r=subprocess.run(['git','-C',str(repo),*args],capture_output=True,text=True)
    data[name]=r.stdout.strip() if r.returncode==0 else 'unavailable'
r=subprocess.run(['git','-C',str(repo),'diff','--binary'],capture_output=True)
data['fastlio_diff_sha256']=hashlib.sha256(r.stdout).hexdigest() if r.returncode==0 else 'unavailable'
mapping=repo/'src/laserMapping.cpp'
data['fastlio_mapping_source_sha256']=hashlib.sha256(mapping.read_bytes()).hexdigest() if mapping.exists() else 'not_present'
payload=json.dumps(data,ensure_ascii=False,indent=2)+'\n'
for path,text in ((out,payload),(out.with_suffix('.h'),'#pragma once\nstatic const char* const robot_health_source_manifest = '+json.dumps(payload,ensure_ascii=True)+';\n')):
    if not path.exists() or path.read_text()!=text:path.write_text(text)
