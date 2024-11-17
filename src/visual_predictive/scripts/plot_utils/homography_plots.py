import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rc
import subprocess
import os

rc('text', usetex=True)



legendfontsize = 18
ticksfontsize  = 18 
axisfontsize   = ticksfontsize + 3
graphwidth     = 2
trajwidth      = 1

def plot_homography_two_experiments(file_path_1,file_path_2,prefix_name):
    # Leer el archivo como un DataFrame
   column_names = ['time', 'h11_p', 'h11_c', 'h12_p', 'h12_c', 'h13_p', 'h13_c', 'h31_p', 'h31_c', 'h32_p', 'h32_c', 'h33_p', 'h33_c', 'v', 'w']
   data_1 = pd.read_csv(file_path_1, delim_whitespace=True, names=column_names)
   data_2 = pd.read_csv(file_path_2, delim_whitespace=True, names=column_names)


   fig = plt.figure()
   ax = fig.add_subplot(2, 1, 1)
   ax2 = fig.add_subplot(2, 1, 2)
   colors = ['red','green','blue','magenta','cyan','yellow']
   index  = ['h_{11}','h_{12}','h_{13}','h_{31}','h_{32}','h_{33}']
   #cols   = [1,3,5,7,9,11]
   cols   = ['h11_c','h12_c','h13_c','h31_c','h32_c','h33_c']
   for i,col in enumerate(cols):
      #ax.plot(data.iloc[:, col], color=colors[i], label= r'%s predicted'%index[i],linestyle='--')
      ax.plot(data_1['time'].values,data_1[col].values, color=colors[i], label= r'$%s$'%index[i],linestyle='-', linewidth=graphwidth)
      ax2.plot(data_2['time'].values,data_2[col].values, color=colors[i], label= r'$%s$'%index[i],linestyle='-', linewidth=graphwidth)

   ax.set_xlabel('time [$s$]',size=axisfontsize)
   ax.set_ylabel('Homography  elements',size=axisfontsize)
   ax.legend(loc='best',fontsize=legendfontsize-1, ncol=4)
   ax.grid(True, linewidth=trajwidth)
   ax.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3)  

   ax2.set_xlabel('time [$s$]',size=axisfontsize)
   ax2.set_ylabel('Homography  elements',size=axisfontsize)
   ax2.legend(loc='best',fontsize=legendfontsize-1, ncol=4)
   ax2.grid(True, linewidth=trajwidth)
   ax2.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3) 

   fig.subplots_adjust(wspace=0.3, hspace=0.3)
   fig.set_size_inches(1.35*fig.get_size_inches())
   fig.savefig('{0}_comparacion_homography.pdf'.format(prefix_name), bbox_inches='tight', dpi=300)

def plot_visual_features_controls(file_path,prefix_name):

   # Leer el archivo como un DataFrame
   column_names = ['time', 'h11_p', 'h11_c', 'h12_p', 'h12_c', 'h13_p', 'h13_c', 'h31_p', 'h31_c', 'h32_p', 'h32_c', 'h33_p', 'h33_c', 'v', 'w']
   data = pd.read_csv(file_path, delim_whitespace=True, names=column_names)


   fig = plt.figure()
   ax = fig.add_subplot(2, 1, 1)
   colors = ['red','green','blue','magenta','cyan','yellow']
   index  = ['h_{11}','h_{12}','h_{13}','h_{31}','h_{32}','h_{33}']
   #cols   = [1,3,5,7,9,11]
   cols   = ['h11_c','h12_c','h13_c','h31_c','h32_c','h33_c']
   for i,col in enumerate(cols):
      #ax.plot(data.iloc[:, col], color=colors[i], label= r'%s predicted'%index[i],linestyle='--')
      ax.plot(data['time'].values,data[col].values, color=colors[i], label= r'$%s$'%index[i],linestyle='-', linewidth=graphwidth)

   ax.set_xlabel('time [$s$]',size=axisfontsize)
   ax.set_ylabel('Homography  elements',size=axisfontsize)
   ax.legend(loc='best',fontsize=legendfontsize-1, ncol=4)
   ax.grid(True, linewidth=trajwidth)
   ax.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3)

   # fig.subplots_adjust(wspace=0.3, hspace=0.3)
   # fig.set_size_inches(1.25*fig.get_size_inches())
   # fig.savefig('{0}_homograpy_elements.pdf'.format('Experiment_1'), bbox_inches='tight', dpi=300)


   #fig = plt.figure()
   ax = fig.add_subplot(2, 1, 2)
   ax.plot(data['time'].values,data['v'].values,label=r"$v \; [\frac{m}{s}$]")
   ax.plot(data['time'].values,data['w'].values,label=r" $\omega [\frac{rad}{s}$]")
   ax.grid(True, linewidth=trajwidth)
   ax.legend(loc='best',fontsize=legendfontsize-1, ncol=2)
   ax.set_xlabel(r"time [$s$]",size=axisfontsize)
   ax.set_ylabel(r"Controls",size=axisfontsize)
   ax.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3)

   fig.subplots_adjust(wspace=0.3, hspace=0.3)
   fig.set_size_inches(1.35*fig.get_size_inches())
   fig.savefig('{0}_controls_homography.pdf'.format(prefix_name), bbox_inches='tight', dpi=300)

def video_of_detailed_visual_features(file_path, prefix_name):
   # Leer el archivo como un DataFrame
   column_names = ['time', 'h11_p', 'h11_c', 'h12_p', 'h12_c', 'h13_p', 'h13_c', 'h31_p', 'h31_c', 'h32_p', 'h32_c', 'h33_p', 'h33_c', 'v', 'w']
   data = pd.read_csv(file_path, delim_whitespace=True, names=column_names)

   fig = plt.figure()
   axis = fig.add_subplot(1, 1, 1)
   fig.set_size_inches(1.25*fig.get_size_inches())

   colors = ['red','green','blue','magenta','cyan','yellow']
   index  = ['h_{11}','h_{12}','h_{13}','h_{31}','h_{32}','h_{33}']
   #cols   = [1,3,5,7,9,11]
   cols   = ['h11_c','h12_c','h13_c','h31_c','h32_c','h33_c']


   files = []
   for i in range(1,len(data['time'].values),5):
      #axis.plot(data['time'].values[i],expected_values_1, color='black',  linestyle='--', linewidth=graphwidth)
      if(i==1):
         axis.legend(loc='best',fontsize=legendfontsize-3,ncol=4) 
      #axis.plot(expected_values_2, color='black', label='reference', linestyle='--', linewidth=graphwidth)
      axis.grid(True, linewidth=graphwidth)
      axis.set_xlabel("Iterations",size=axisfontsize)
      axis.set_ylabel("Homography matrix elements",size=axisfontsize)
      axis.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3) 
      axis.set_ylim(-1.2,1.2)
      axis.set_xlim(-0.2,data['time'].values[-1])

      axis.plot(data['time'].values[0:i],data[cols[0]].values[0:i], color=colors[0], label=r'$%s$'%('h_{11}'), linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data[cols[1]].values[0:i], color=colors[1], label=r'$%s$'%('h_{12}'), linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data[cols[2]].values[0:i], color=colors[2], label=r'$%s$'%('h_{13}'), linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data[cols[3]].values[0:i], color=colors[3], label=r'$%s$'%('h_{32}'), linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data[cols[4]].values[0:i], color=colors[4], label=r'$%s$'%('h_{33}'), linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data[cols[5]].values[0:i], color=colors[5], label=r'$%s$'%('h_{31}'), linestyle='-', linewidth=graphwidth)

      if(i==1):
         axis.legend(loc='best',fontsize=legendfontsize-3,ncol=4)  
      fname = '_tmp%05d.png' % i
      print('Saving frame', fname)
      fig.savefig(fname)
      files.append(fname)

   print('Making movie animation.mpg - this may take a while')
   subprocess.call("mencoder 'mf://_tmp*.png' -mf type=png:fps=15 -ovc lavc "
                    "-lavcopts vcodec=wmv2 -oac copy -o {0}_visual_features_animation.mpg".format(prefix_name), shell=True)

    # cleanup
   for fname in files:
      os.remove(fname)

def video_of_controls(file_path, prefix_name):
   # Leer el archivo como un DataFrame
   column_names = ['time', 'h11_p', 'h11_c', 'h12_p', 'h12_c', 'h13_p', 'h13_c', 'h31_p', 'h31_c', 'h32_p', 'h32_c', 'h33_p', 'h33_c', 'v', 'w']
   data = pd.read_csv(file_path, delim_whitespace=True, names=column_names)

   fig = plt.figure()
   axis = fig.add_subplot(1, 1, 1)
   fig.set_size_inches(1.25*fig.get_size_inches())

   colors = ['red','green','blue','magenta','cyan','yellow']


   files = []
   for i in range(1,len(data['time'].values),5):
      #axis.plot(data['time'].values[i],expected_values_1, color='black',  linestyle='--', linewidth=graphwidth)
      if(i==1):
         axis.legend(loc='best',fontsize=legendfontsize-3,ncol=4) 
      #axis.plot(expected_values_2, color='black', label='reference', linestyle='--', linewidth=graphwidth)
      axis.grid(True, linewidth=graphwidth)
      axis.set_xlabel(r"time [$s$]",size=axisfontsize)
      axis.set_ylabel(r"Controls",size=axisfontsize)
      axis.tick_params(labelsize=ticksfontsize,labelcolor='black',pad=3) 
      axis.set_ylim(-0.4,0.4)
      axis.set_xlim(-0.2,data['time'].values[-1])

      axis.plot(data['time'].values[0:i],data['v'].values[0:i],label=r"$v \; [\frac{m}{s}$]", color='blue', linestyle='-', linewidth=graphwidth)
      axis.plot(data['time'].values[0:i],data['w'].values[0:i],label=r" $\omega [\frac{rad}{s}$]", color='orange', linestyle='-', linewidth=graphwidth)


      if(i==1):
         axis.legend(loc='best',fontsize=legendfontsize-3,ncol=4)  
      fname = '_tmp%05d.png' % i
      print('Saving frame', fname)
      fig.savefig(fname)
      files.append(fname)

   print('Making movie animation.mpg - this may take a while')
   subprocess.call("mencoder 'mf://_tmp*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=wmv2 -oac copy -o {0}_controls_animation.mpg".format(prefix_name), shell=True)

    # cleanup
   for fname in files:
      os.remove(fname)


if __name__ == '__main__':

   #Print controls and homograpy elements
   # prefix_name = "Experiment_1"
   # file_path = 'data_experiment_1_py.txt'
   # #plot_visual_features_controls(file_path,prefix_name)
   # #video_of_detailed_visual_features(file_path, prefix_name)
   # video_of_controls(file_path, prefix_name)

   # prefix_name = "Experiment_2"
   # file_path = 'data_experiment_2_py.txt'
   # #plot_visual_features_controls(file_path,prefix_name)
   # #video_of_detailed_visual_features(file_path, prefix_name)
   # video_of_controls(file_path, prefix_name)

   # prefix_name = "Experiment_3"
   # file_path = 'data_experiment_3_py.txt'
   # #plot_visual_features_controls(file_path,prefix_name)
   # #video_of_detailed_visual_features(file_path, prefix_name)
   # video_of_controls(file_path, prefix_name)

   #prefix_name = "Experiment_tiago"
   #file_path = 'data_simulation_tiago.txt'
   # #plot_visual_features_controls(file_path,prefix_name)
   # video_of_detailed_visual_features(file_path, prefix_name)
   #video_of_controls(file_path, prefix_name)

   prefix_name = "Experimento_1_pioneer"
   file_path = "data_pioneer_1.txt"
   #plot_visual_features_controls(file_path,prefix_name)
   #video_of_detailed_visual_features(file_path, prefix_name)
   video_of_controls(file_path, prefix_name)

   #prefix_name = "Experimento_comparacion_pioneer"
   #file_path_1 = "data_pioneer_1.txt"
   #file_path_2 = "data_pioneer_2.txt"
   #plot_homography_two_experiments(file_path_1,file_path_2,prefix_name)






