"""No ROS node/publisher is needed: mock the bag decoder and check exporter contract."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as S
from unittest import TestCase, mock
import sys
import tempfile

import export_bag as e


def message(frame, t, **details):
    return S(header=S(frame_id=frame, stamp=S(sec=t, nanosec=0)), **details)


def streams(truth_start=1, child='base_link'):
    q=S(w=1.,x=0.,y=0.,z=0.)
    xyz=S(x=0.,y=0.,z=0.)
    p=S(position=xyz,orientation=q)
    base=[('/map',message('map',0,info=S(width=2,height=2,resolution=.1,origin=p),data=[0,0,0,100])),
          ('/tf_static',S(transforms=[S(header=S(frame_id='base_link'),child_frame_id='lidar_link',
                                        transform=S(translation=xyz,rotation=q))]))]
    for t in (1,2):
        base.extend([
            ('/sensors/wheel_odom',message('odom',t,child_frame_id='base_link',twist=S(twist=S(linear=xyz,angular=xyz)))),
            ('/sensors/scan',message('lidar_link',t,angle_min=0.,angle_increment=.1,range_min=.01,range_max=10.,ranges=[1.,2.]))])
    truth=[(e.GT_TOPIC,message('map',t,child_frame_id=child,pose=S(pose=p))) for t in (truth_start,truth_start+1)]
    return base,truth


class SidecarExportTest(TestCase):
    def export(self, root, split, truth_start=1, child='base_link', pilot=True):
        parent=root/('hw1-pilot-test' if pilot else 'ordinary')
        left,right=parent/'inputs',parent/'ground_truth'
        for p in (left,right):
            p.mkdir(parents=True,exist_ok=True)
            (p/'data.db3').write_bytes(p.name.encode())
        world=root/'world.wbt';world.write_text('fixture')
        output=root/'out'
        argv=['export_bag.py',str(left),str(output),'--ground-truth-bag',str(right),
              '--world','fixture','--world-file',str(world),'--trajectory','route','--scenario','nominal',
              '--split',split,'--initial','-8','0','0','--gt-to-map','0','0','0']
        with mock.patch.object(sys,'argv',argv),mock.patch.object(e,'read_bag',side_effect=streams(truth_start,child)) as dec:
            e.main()
        self.assertTrue(dec.call_args_list[0].kwargs['exclude_embedded'])
        self.assertTrue(dec.call_args_list[1].kwargs['sidecar'])
        return output

    def test_pilot_remains_ineligible_and_hashes_are_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            output=self.export(Path(directory),'pilot')
            m=json.loads((output/'manifest.json').read_text())
            self.assertEqual(m['source'],'recorded_pilot')
            self.assertFalse(m['eligible_for_scored_pipeline'])
            self.assertEqual(m['counts']['ground_truth'],2)
            self.assertEqual(m['bag_file_sha256']['data.db3'],hashlib.sha256(b'inputs').hexdigest())
            self.assertEqual(m['ground_truth_bag_file_sha256']['data.db3'],hashlib.sha256(b'ground_truth').hexdigest())
            text=(output/'events.txt').read_text()
            self.assertEqual([line[0] for line in text.splitlines()],['W','S','W','S'])
            self.assertEqual(len((output/'ground_truth.csv').read_text().splitlines()),3)

    def test_known_pilot_cannot_be_relabelled_and_overlap_is_required(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            with self.assertRaisesRegex(ValueError,'Known pilot'):
                self.export(root,'train')
            with self.assertRaisesRegex(ValueError,'overlapping'):
                self.export(root,'pilot',10)
            with self.assertRaisesRegex(ValueError,'map -> base_link'):
                self.export(root,'pilot',1,'wheel_link')
            self.assertFalse((root/'out').exists())

    def test_ordinary_sidecar_preserves_recorded_source(self):
        with tempfile.TemporaryDirectory() as directory:
            output=self.export(Path(directory),'validation',pilot=False)
            m=json.loads((output/'manifest.json').read_text())
            self.assertEqual(m['source'],'recorded_rosbag')
            self.assertTrue(m['eligible_for_scored_pipeline'])
