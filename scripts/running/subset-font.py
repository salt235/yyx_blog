"""Build the running page's Chinese web fonts from the Maple Mono v7.9 release."""
import argparse
from pathlib import Path

from fontTools import subset
from fontTools.ttLib import TTFont

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--font-dir', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
output = root / 'docs/.vuepress/public/fonts/maple-mono'
ranges = [(0x2E80, 0x30FF), (0xFF00, 0xFFEF)]
common = set()
# Include common Chinese characters so most future labels and locations work too.
for lead in range(0xB0, 0xD8):
    for trail in range(0xA1, 0xFF):
        try:
            common.update(map(ord, bytes([lead, trail]).decode('gb2312')))
        except UnicodeDecodeError:
            pass
for path in [*(root / 'docs/.vuepress/theme/components/running').glob('*.vue'),
             root / 'docs/.vuepress/public/running-data/activities.json']:
    common.update(ord(c) for c in path.read_text(encoding='utf8') if ord(c) > 127)

for style in ['Regular', 'SemiBold']:
    font = TTFont(args.font_dir / f'MapleMono-NF-CN-{style}.ttf', recalcTimestamp=False)
    keep = {code for code in font.getBestCmap()
            if code in common or any(start <= code <= end for start, end in ranges)}
    options = subset.Options()
    options.hinting = False
    options.name_IDs = ['*']
    options.name_legacy = True
    options.name_languages = ['*']
    subsetter = subset.Subsetter(options=options)
    subsetter.populate(unicodes=keep)
    subsetter.subset(font)
    font.flavor = 'woff2'
    target = output / f'MapleMonoCN-{style}.woff2'
    font.save(target)
    print(f'{target.name}: {target.stat().st_size:,} bytes')
