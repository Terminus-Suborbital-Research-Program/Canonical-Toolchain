#!/bin/python3
import json
import argparse
import matplotlib.pyplot as plt

class StatePlotter:
    def __init__(self, minicom_file_name):
        self.minicom_file_name = minicom_file_name


    def parse(self):
        print("Parsing: " + self.minicom_file_name)

        with open(self.minicom_file_name, 'r') as fp:
            fc = fp.readlines()

            self.count = []
            self.ab_data = [[],[],[]]
            self.dwb_data = [[],[],[]]
            self.wb_data = [[],[],[]]
            self.e_data = [[],[],[]]
            self.qb_data = [[],[],[]]
            self.m_data = [[],[],[]]
            for i, jc_line in enumerate(fc):
                jc = json.loads(jc_line)
                ab = jc['ab']
                    


                self.ab_data[0].append(jc['ab'][0])
                self.ab_data[1].append(jc['ab'][1])
                self.ab_data[2].append(jc['ab'][2])
                self.dwb_data[0].append(jc['dwb'][0])
                self.dwb_data[1].append(jc['dwb'][1])
                self.dwb_data[2].append(jc['dwb'][2])
                self.wb_data[0].append(jc['wb'][0])
                self.wb_data[1].append(jc['wb'][1])
                self.wb_data[2].append(jc['wb'][2])
                self.e_data[0].append(jc['e'][0])
                self.e_data[1].append(jc['e'][1])
                self.e_data[2].append(jc['e'][2])
                self.qb_data[0].append(jc['qb'][0])
                self.qb_data[1].append(jc['qb'][1])
                self.qb_data[2].append(jc['qb'][2])

                if abs(jc['m'][0]) < 50.0 and abs(jc['m'][1]) < 50.0 and abs(jc['m'][2]) < 50.0:
                    print(jc['m'][0], jc['m'][1], jc['m'][2])
                    self.count.append(i*0.1)
                    self.m_data[0].append(jc['m'][0])
                    self.m_data[1].append(jc['m'][1])
                    self.m_data[2].append(jc['m'][2])



    def plot(self):
        fig, ax = plt.subplots(nrows=3, ncols=1)  # create figure & 1 axis

        ax[0].plot(self.count, self.m_data[0])
        ax[0].set_ylabel("X-Axis [uT]")
        ax[1].plot(self.count, self.m_data[1])
        ax[1].set_ylabel("Y-Axis [uT]")
        ax[2].plot(self.count, self.m_data[2])
        ax[2].set_ylabel("Z-Axis [uT]")

        fig.supxlabel('Time [s]')
        fig.suptitle("Magnetometer Data")
        fig.savefig('magnetometer.png')
        plt.close(fig)

    def run(self):
        self.parse()
        self.plot()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_name", help="Minicom File with Telemetry...")
    args = parser.parse_args()

    state_plotter = StatePlotter(args.file_name)
    state_plotter.run()

    
if __name__ == '__main__':
    main()