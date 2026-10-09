from pathlib import Path
import re
from collections import Counter

p=Path('docs/AVIATOR故障编码规范.md')
t=p.read_text(encoding='utf-8')
section=t.split('## 5. 故障码表\n',1)[1].split('## 6. RS422',1)[0]
codes=[]
for row in section.splitlines():
    if row.startswith('|'):
        codes.extend(int(c.strip()) for c in row.split('|') if re.fullmatch(r'\d{4}',c.strip()))
assert len(codes)==len(set(codes)), Counter(codes)
assert all(1000<=c<=9999 and 1<=c//1000<=9 and c//100%10<=3 and c%100!=0 for c in codes)
assert set(c//1000 for c in codes)==set(range(1,10))
assert all(1000+c in codes for c in codes if c//1000 in (1,3))
assert all(c in codes for c in (1204,2204,3207,7105,9301,8299,9399))
assert all(c not in codes for c in (1000,1401,4097,9000,9999,10000,65535))

vector_section=t.split('### 6.1 两字节编码',1)[1].split('### 6.2',1)[0]
vectors=re.findall(r'^\| (\d+) \| [^|]+ \| `(0x[0-9A-F]+)` \| `([0-9A-F ]+)` \|$', vector_section, re.M)
assert len(vectors)==7
for n,h,b in vectors:
    assert int(n)==int(h,16)==int.from_bytes(bytes.fromhex(b),'little'), (n,h,b)

snippet=re.search(r'```python\n(.*?)\n```',t,re.S).group(1)
ns={}
exec(compile(snippet,str(p),'exec'),ns)
for c in codes:
    assert ns['decode_raw_code'](ns['encode_code'](c,set(codes)))==c

width=None
for i,l in enumerate(t.splitlines(),1):
    if not l.startswith('|'):
        width=None
        continue
    current=len(l.split('|'))
    if width is None: width=current
    assert current==width,(i,current,width)

for target in re.findall(r'\]\(([^)#]+)(?:#[^)]*)?\)',t):
    assert (p.parent/target).exists(),target

print('Validated:',len(codes),'assigned codes;',len(vectors),'little-endian vectors; sample and full code roundtrips; Markdown tables and local links.')
print('By category:',dict(sorted(Counter(c//1000 for c in codes).items())))
