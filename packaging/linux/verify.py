#!/usr/bin/env python3
"""Validate the actual release payload and relocated native model entrypoint"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile


def digest(stream):
    result=hashlib.sha256()
    for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def validate_payload(payload):
    receipt=json.loads((payload/'build-receipt.json').read_text())
    for name, expected in receipt['files'].items():
        target=(payload/name).resolve()
        if not target.is_relative_to(payload.resolve()) or not target.is_file():
            raise ValueError('Missing payload resource: '+name)
        with target.open('rb') as stream:
            if digest(stream)!=expected:raise ValueError('Payload hash mismatch: '+name)
    for name in ('pybind11.txt','pinocchio.txt','coal.txt','eigenpy.txt'):
        if not (payload/'licenses'/name).is_file():raise ValueError('Missing dependency notice: '+name)
    env=dict(os.environ)
    for name in ('LD_LIBRARY_PATH','PYTHONPATH','SERIAL_ARM_RESOURCE_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH'):
        env.pop(name,None)
    native=payload/'native'
    config=native/'share/dm_arm_description/config/core/gray.yaml'
    result=subprocess.run([str(native/'bin/serial_arm_model_probe'),'--config',str(config)],
        env=env,capture_output=True,text=True,timeout=30)
    if result.returncode:raise ValueError('Relocated model probe failed: '+result.stderr[-1800:])
    model=json.loads(result.stdout)
    if not model.get('ok'):raise ValueError('Relocated model probe returned failure')
    return receipt


def check_asar(root, asar):
    script=r"""
const fs=require('node:fs'), path=require('node:path');
const root=process.argv[1],archive=process.argv[2];
const asar=require(path.join(root,'packaging/linux/node_modules/@electron/asar'));
function visit(directory) {
  for(const entry of fs.readdirSync(directory,{withFileTypes:true})) {
    const file=path.join(directory,entry.name);
    if(entry.isDirectory())visit(file);
    else {
      const relative=path.relative(path.join(root,'apps/launcher'),file);
      const actual=asar.extractFile(archive,relative);
      if(!actual.equals(fs.readFileSync(file)))throw new Error('ASAR source mismatch: '+relative);
    }
  }
}
visit(path.join(root,'apps/launcher/desktop'));
visit(path.join(root,'apps/launcher/renderer'));
"""
    subprocess.run(['node','-e',script,str(root),str(asar)],check=True,timeout=60)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--deb',type=Path,required=True)
    parser.add_argument('--appimage',type=Path,required=True)
    args=parser.parse_args()
    from stage import read_manifest
    read_manifest(args.root.resolve())
    installation=json.loads((args.root/'.install/manifest.json').read_text())
    with tempfile.TemporaryDirectory(prefix='serialarm-release-check-') as temporary:
        folder=Path(temporary)
        # dpkg extracts generated assets into an isolated directory, never into the host installation
        deb=folder/'deb'
        subprocess.run(['dpkg-deb','--extract',str(args.deb.resolve()),str(deb)],check=True,timeout=120)
        deb_app=next((deb/'opt').iterdir())
        deb_payload=deb_app/'resources/serial-arm'
        receipt=validate_payload(deb_payload)
        if (receipt['source']['native']!=installation['native_receipt']['source_sha256'] or
                receipt['source']['gui']!=installation['gui_source_sha256']):
            raise ValueError('Release payload does not match the verified current installation')
        check_asar(args.root.resolve(),deb_app/'resources/app.asar')
        # Runtime binaries may contain the SquashFS magic in their own code or strings
        offset_result=subprocess.run([str(args.appimage.resolve()),'--appimage-offset'],
                                     capture_output=True,text=True,timeout=15,check=True)
        offset=int(offset_result.stdout.strip())
        with args.appimage.open('rb') as stream:
            stream.seek(offset)
            if stream.read(4)!=b'hsqs':raise ValueError('AppImage filesystem is missing')
        appimage=folder/'appimage'
        unpack=subprocess.run(['unsquashfs','-no-progress','-o',str(offset),'-d',str(appimage),str(args.appimage.resolve())],
                              capture_output=True,text=True,timeout=120)
        if unpack.returncode:raise ValueError('AppImage extraction failed: '+unpack.stderr[-1200:])
        other=validate_payload(appimage/'resources/serial-arm')
        if receipt!=other:raise ValueError('deb and AppImage contain different native payloads')
        if (deb_app/'resources/app.asar').read_bytes()!=(appimage/'resources/app.asar').read_bytes():
            raise ValueError('deb and AppImage contain different GUI payloads')
        print('RELEASE_PAYLOAD_PASS')

if __name__=='__main__':main()
