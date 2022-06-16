#!/usr/bin/env python
# coding: utf-8

# In[1]:


import copy
import numpy as np

import argoverse
from argoverse.data_loading.argoverse_tracking_loader import ArgoverseTrackingLoader

from argoverse.map_representation.map_api import ArgoverseMap


# In[ ]:


from argoverse.evaluation.competition_util import generate_tracking_zip

data_dir = '/data/results/results_tracking_test_cbgs'
output_dir = '/data/final'

generate_tracking_zip(data_dir,output_dir)

