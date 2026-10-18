"""Named joint trajectories stored under the owning Profile .serialarm directory."""
from __future__ import annotations
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import uuid
import zipfile


def safe_profile(name):
    return hashlib.sha256(name.encode()).hexdigest()[:18]


class TrajectoryLibrary:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.legacy_root = self.root / '.install' / 'teach-replay'

    @staticmethod
    def _identity(scope):
        return {key: value for key, value in scope.items() if key != '_storage_home'}

    def folder(self, scope):
        home = scope.get('_storage_home')
        if not home or not Path(home).is_absolute():
            raise ValueError('无法确定当前 Profile 的存储位置')
        return Path(home).resolve() / '.serialarm' / 'teach-replay'

    @staticmethod
    def validate_name(name):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80 or any(c in name for c in '\\/:*?"<>|\r\n\0'):
            raise ValueError('轨迹名称长度必须为 1 至 80 个字符且不能包含路径特殊字符')
        return name.strip()

    @classmethod
    def _slug(cls, name):
        name = cls.validate_name(name)
        value = re.sub(r'\s+', '-', name).strip('. -')[:60]
        if value in ('', '.', '..') or value.startswith('.'):
            raise ValueError('轨迹名称不能是隐藏目录或空目录')
        return value

    @classmethod
    def _new_folder(cls, base, name):
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        target = base / (cls._slug(name) + '-' + stamp)
        target.mkdir(parents=True, exist_ok=False)
        return target

    @staticmethod
    def scope(info, repository_root=None):
        core = info['resources']['core']
        import yaml
        from profile_storage import profile_home
        conf = yaml.safe_load(Path(core).read_text())
        model = conf['model']
        urdf = (Path(core).parent / model['urdf_path']).resolve()
        if not urdf.is_file():
            raise ValueError('当前 URDF 不存在')
        hardware_path = Path(info['resources'].get('hardware', ''))
        hardware = yaml.safe_load(hardware_path.read_text()) if hardware_path.is_file() else {}
        calibration = conf.get('calibration') or {}
        mapping = {'calibration': calibration,
                   'hardware_plugin': info.get('hardware_plugin'),
                   'actuators': (hardware.get('damiao') or {}).get('actuators')}
        mapping_hash = hashlib.sha256(json.dumps(mapping, sort_keys=True, ensure_ascii=False, default=str).encode()).hexdigest()
        root = Path(repository_root).resolve() if repository_root is not None else Path(__file__).resolve().parents[3]
        return dict(profile=info['profile'], joints=model['joint_names'],
                    urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                    mapping_sha256=mapping_hash,
                    base_frame=model['base_frame'], tool_frame=model['tool_frame'],
                    _storage_home=str(profile_home(info, root)))

    @staticmethod
    def validate_csv(path, joints):
        with Path(path).open(newline='', encoding='utf-8') as handle:
            reader = csv.reader(handle)
            dt = next(reader, None)
            if not dt or len(dt) != 2 or dt[0] != 'sample_dt':
                raise ValueError('轨迹缺少 sample_dt')
            interval = float(dt[1])
            if not math.isfinite(interval) or not .001 <= interval <= 1.0:
                raise ValueError('轨迹采样周期无效')
            if next(reader, None) != joints:
                raise ValueError('轨迹关节名称或顺序与当前机械臂不一致')
            count = 0
            start = end = None
            for row in reader:
                if not row or len(row) != len(joints):
                    raise ValueError('轨迹存在空行或缺失的关节列')
                values = [float(x) for x in row]
                if not all(math.isfinite(x) for x in values):
                    raise ValueError('轨迹包含非法数值')
                if start is None:
                    start = values
                end = values
                count += 1
                if count > 300000:
                    raise ValueError('轨迹样本数量超限')
        if count < 20:
            raise ValueError('有效轨迹需要至少 20 个采样点')
        return {'samples':count, 'duration_s':round((count-1)*interval,3), 'start':start,'end':end}

    def _roots(self, scope):
        return [self.folder(scope), self.legacy_root / safe_profile(scope['profile'])]

    def list(self, scope):
        found = []
        for base in self._roots(scope):
            if not base.is_dir():
                continue
            for meta_path in base.glob('*/metadata.json'):
                try:
                    data = json.loads(meta_path.read_text(encoding='utf-8'))
                    if data.get('scope') != self._identity(scope) or not (meta_path.parent/'trajectory.csv').is_file():
                        continue
                    found.append({**data, 'directory':str(meta_path.parent.resolve())})
                except (OSError, ValueError):
                    continue
        return sorted(found, key=lambda x: x.get('created_at', x.get('id','')), reverse=True)

    def ensure(self, scope, record_id):
        if not isinstance(record_id,str) or not re.fullmatch(r'[0-9a-f]{32}',record_id):
            raise ValueError('轨迹 ID 无效')
        for base in self._roots(scope):
            if not base.is_dir():
                continue
            for path in base.glob('*/metadata.json'):
                try:
                    info = json.loads(path.read_text(encoding='utf-8'))
                    if info.get('id') == record_id and info.get('scope') == self._identity(scope):
                        return path.parent, info
                except (OSError,ValueError):
                    continue
        raise ValueError('轨迹不存在，或来源与当前机械臂不匹配')

    def save(self, scope, name, native):
        name = self.validate_name(name)
        base = self.folder(scope);base.mkdir(parents=True,exist_ok=True)
        key = uuid.uuid4().hex
        target = self._new_folder(base,name)
        try:
            native.request('teach_replay_save',{'path':str(target/'trajectory.csv')},timeout=15)
            summary = self.validate_csv(target/'trajectory.csv',scope['joints'])
            meta = {'id':key,'name':name,'scope':self._identity(scope),
                    'created_at':datetime.now().isoformat(timespec='seconds'),**summary}
            (target/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
            return {**meta,'directory':str(target)}
        except Exception:
            shutil.rmtree(target,ignore_errors=True)
            raise

    def load(self,scope,record_id,native):
        folder,meta=self.ensure(scope,record_id)
        native.request('teach_replay_load',{'path':str(folder/'trajectory.csv')},timeout=15)
        return {**meta,'directory':str(folder)}

    def rename(self,scope,record_id,name):
        folder,meta=self.ensure(scope,record_id)
        name=self.validate_name(name)
        new_folder=folder
        if folder.parent.resolve()==self.folder(scope).resolve():
            old_stamp=re.search(r'(\d{8}-\d{6}-\d{6})(?:-[0-9]+)?$', folder.name)
            stamp=old_stamp.group(1) if old_stamp else datetime.now().strftime('%Y%m%d-%H%M%S-%f')
            target=folder.parent/(self._slug(name)+'-'+stamp)
            if target!=folder:
                if target.exists():raise ValueError('同名轨迹文件夹已存在')
                folder.rename(target);new_folder=target
        meta['name']=name
        (new_folder/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
        return {**meta,'directory':str(new_folder)}

    def remove(self,scope,record_id):
        folder,_=self.ensure(scope,record_id)
        shutil.rmtree(folder)
        return {'removed':record_id}

    def export(self,scope,record_id,destination):
        folder,meta=self.ensure(scope,record_id)
        output=Path(destination).expanduser().resolve() if destination else self.folder(scope).parent/'exports'/'teach-replay'
        if destination and not output.is_dir():raise ValueError('请选择存在的导出目录')
        output.mkdir(parents=True,exist_ok=True)
        target=output/(self._slug(meta['name'])+'-'+meta['id']+'.zip')
        with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED) as archive:
            archive.write(folder/'metadata.json','metadata.json')
            archive.write(folder/'trajectory.csv','trajectory.csv')
        return {'path':str(target),'directory':str(output)}

    def import_file(self,scope,source):
        source=Path(source).expanduser().resolve()
        if not source.is_file() or source.stat().st_size>30_000_000:raise ValueError('导入文件不存在或超过 30 MB')
        if source.suffix.lower()!='.zip':
            raise ValueError('仅支持导入 SerialArm-Core 导出的轨迹 ZIP')
        with zipfile.ZipFile(source) as archive:
            names=archive.namelist()
            if sorted(names)!=['metadata.json','trajectory.csv']:
                raise ValueError('轨迹 ZIP 必须只包含 metadata.json 和 trajectory.csv')
            if any(archive.getinfo(k).file_size>30_000_000 for k in names):raise ValueError('轨迹压缩包解压大小超限')
            meta=json.loads(archive.read('metadata.json'))
            if meta.get('scope')!=self._identity(scope):raise ValueError('轨迹不匹配当前 Profile、控制链或 URDF')
            base=self.folder(scope);base.mkdir(parents=True,exist_ok=True)
            name=self.validate_name(meta.get('name'))
            key=uuid.uuid4().hex
            target=self._new_folder(base,name)
            try:
                (target/'trajectory.csv').write_bytes(archive.read('trajectory.csv'))
                checked=self.validate_csv(target/'trajectory.csv',scope['joints'])
                record={'id':key,'name':name,'scope':self._identity(scope),
                        'created_at':datetime.now().isoformat(timespec='seconds'),**checked}
                (target/'metadata.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
                return {**record,'directory':str(target)}
            except Exception:
                shutil.rmtree(target,ignore_errors=True)
                raise
