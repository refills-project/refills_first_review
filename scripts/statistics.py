#!/usr/bin/env python
from __future__ import division
import json
import numpy as np
import pylab as plt
import matplotlib.mlab as mlab
from glob import glob
from collections import defaultdict


class Evaluation(object):
    def __init__(self):
        self.expected_fn = 23
        self.image_id = 0

    def load_scanning_data(self):
        datas = {}
        self.count = {}
        for i, f in enumerate(glob('../data/scanning_data*.json')):
            print('loading: {}'.format(f))
            data = json.load(open(f))
            better_data = {}
            for barcode, pose_stampeds in data.items():
                l = []
                for pose_stamped in pose_stampeds:
                    l.append(np.array([pose_stamped['pose']['position']['x'],
                                       pose_stamped['pose']['position']['y'],
                                       pose_stamped['pose']['position']['z']]))
                better_data[barcode[1:-1]] = np.median(l, axis=0)
                self.count[i, barcode[1:-1]] = len(l)
            datas[i] = better_data
        return datas

    def load_ground_truth(self):
        data = json.load(open('../data/ground_truth.json'))
        better_data = {}
        for shelf in data.values():
            for floor in shelf:
                for barcode in floor['barcode']:
                    better_data[str(barcode['code'])[:-1]] = np.array([-barcode['pose']['position']['x'],
                                                                       0.515 - barcode['pose']['position']['y'],
                                                                       0.16 + barcode['pose']['position']['z']])
        return better_data

    def tp_distance(self, ground_truth, scanning_datas, tps):
        diffs = []
        diffs_x = []
        diffs_y = []
        diffs_z = []
        for i, scanning_data in scanning_datas.items():
            for tp in tps[i]:
                diff = np.linalg.norm(ground_truth[tp] - scanning_data[tp])
                diff_x = abs(ground_truth[tp][0] - scanning_data[tp][0])
                diff_y = abs(ground_truth[tp][1] - scanning_data[tp][1])
                diff_z = abs(ground_truth[tp][2] - scanning_data[tp][2])
                # if diff > .1:
                #     print(tp)
                #     print(diff)
                diffs.append(diff)
                diffs_x.append(diff_x)
                diffs_y.append(diff_y)
                diffs_z.append(diff_z)
        max_ = max(diffs + diffs_x + diffs_y + diffs_z) + 0.05
        self.show_histogram(diffs, max_, 'euclidean distance of TPs in m')
        self.show_histogram(diffs_x, max_, 'x')
        self.show_histogram(diffs_y, max_, 'y')
        self.show_histogram(diffs_z, max_, 'z')

    def show_histogram(self, x, max_, xlabel='', save=True):

        n, bins, patches = plt.hist(x, 50, range=[0, max_], facecolor='green', alpha=0.75)

        plt.xlabel(xlabel)
        plt.ylabel('number')
        avg_diff = np.mean(x)
        plt.axvline(avg_diff, lw=2)
        plt.text(avg_diff, len(x) / 2, 'avg={:.4f}m'.format(avg_diff))
        plt.axis([0, max_, 0, len(x)])
        plt.grid(True)
        # print('avg diff: {}'.format(np.mean(x)))

        if save:
            plt.savefig('../tex/{}_{}.pdf'.format(self.image_id, xlabel.replace(' ', '_')))
            self.image_id += 1
            plt.close()
        else:
            plt.show()

    def precision(self, tp, fp):
        return tp / (tp + fp)

    def recall(self, tp, fn):
        return tp / (tp + fn)

    def count_barcodes(self, scanning_datas, tp_barcodes):
        tp_counts = []
        fp_counts = []
        for i, scanning_data in scanning_datas.items():
            for barcode in scanning_data:
                if barcode in tp_barcodes[i]:
                    tp_counts.append(self.count[i, barcode])
                else:
                    fp_counts.append(self.count[i, barcode])
        self.show_histogram(tp_counts, 50, 'number of times a tp has been seen')
        print('min a tp barcode has been detected: {}'.format(min(tp_counts)))
        self.show_histogram(fp_counts, 50, 'number of times a fp has been seen')

    def analyse(self):
        with open('../tex/template.tex', 'r') as template_tex:
            with open('../tex/report.tex', 'w') as f:
                for line in template_tex.readlines():
                    if line.startswith('%insert here'):
                        ground_truth = self.load_ground_truth()
                        scanning_datas = self.load_scanning_data()
                        tp_barcodes = defaultdict(list)
                        tp = {}
                        fp = {}
                        fn = {}
                        for i, scanning_data in scanning_datas.items():
                            for scanned_code in scanning_data:
                                if scanned_code in ground_truth.keys() and scanned_code not in tp_barcodes[i]:
                                    tp_barcodes[i].append(scanned_code)
                            tp[i] = len(tp_barcodes[i])
                            fp[i] = len(scanning_data) - len(tp_barcodes[i])
                            fn[i] = len(ground_truth) - len(tp_barcodes[i]) - self.expected_fn
                        avg_tp = sum(tp.values()) / len(tp)
                        avg_fp = sum(fp.values()) / len(fp)
                        avg_fn = sum(fn.values()) / len(fn)
                        self.tp_distance(ground_truth, scanning_datas, tp_barcodes)
                        self.count_barcodes(scanning_datas, tp_barcodes)

                        f.write('\\begin{tabularx}{\\textwidth}{c|c|c|c|c|c}\n')
                        f.write('Experiment \\# & TP & FP & FN & Precision & Recall \\\\\\hline\n')
                        for i, scanning_data in scanning_datas.items():
                            f.write('{} & {} & {} & {} & {:.4} & {:.4} \\\\\n'.format(i, tp[i], fp[i], fn[i],
                                                             self.precision(tp[i], fp[i]),
                                                             self.recall(tp[i], fn[i])))
                        f.write('\\hline\n')
                        f.write('avg & {} & {} & {} & {:.4} & {:.4} \\\\\n'.format(avg_tp, avg_fp, avg_fn,
                                                         self.precision(avg_tp, avg_fp),
                                                         self.recall(avg_tp, avg_fn)))
                        f.write('\\end{tabularx}\n')
                        for pdf in sorted(glob('../tex/*.pdf')):
                            if 'report' not in pdf:
                                f.write('\\includegraphics[height=0.45\\textheight]{{{}}}\n\n'.format(pdf.split('/')[-1]))
                    else:
                        f.write(line)


if __name__ == '__main__':
    e = Evaluation()
    e.analyse()
