¿
¦
^
AssignVariableOp
resource
value"dtype"
dtypetype"
validate_shapebool( 
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
Á
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring ¨
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
-
Tanh
x"T
y"T"
Ttype:

2

VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 "serve*2.9.12unknown8Ã

Adamax/dense_23/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdamax/dense_23/bias/v
}
*Adamax/dense_23/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_23/bias/v*
_output_shapes
:*
dtype0

Adamax/dense_23/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@*)
shared_nameAdamax/dense_23/kernel/v

,Adamax/dense_23/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_23/kernel/v*
_output_shapes

:@*
dtype0

Adamax/dense_22/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_22/bias/v
}
*Adamax/dense_22/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_22/bias/v*
_output_shapes
:@*
dtype0

Adamax/dense_22/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_22/kernel/v

,Adamax/dense_22/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_22/kernel/v*
_output_shapes

:@@*
dtype0

Adamax/dense_21/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_21/bias/v
}
*Adamax/dense_21/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_21/bias/v*
_output_shapes
:@*
dtype0

Adamax/dense_21/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_21/kernel/v

,Adamax/dense_21/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_21/kernel/v*
_output_shapes

:@@*
dtype0

Adamax/dense_20/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_20/bias/v
}
*Adamax/dense_20/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_20/bias/v*
_output_shapes
:@*
dtype0

Adamax/dense_20/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_20/kernel/v

,Adamax/dense_20/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_20/kernel/v*
_output_shapes

:@@*
dtype0

Adamax/dense_19/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_19/bias/v
}
*Adamax/dense_19/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_19/bias/v*
_output_shapes
:@*
dtype0

Adamax/dense_19/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_19/kernel/v

,Adamax/dense_19/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_19/kernel/v*
_output_shapes

:@@*
dtype0

Adamax/dense_18/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_18/bias/v
}
*Adamax/dense_18/bias/v/Read/ReadVariableOpReadVariableOpAdamax/dense_18/bias/v*
_output_shapes
:@*
dtype0

Adamax/dense_18/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@*)
shared_nameAdamax/dense_18/kernel/v

,Adamax/dense_18/kernel/v/Read/ReadVariableOpReadVariableOpAdamax/dense_18/kernel/v*
_output_shapes

:@*
dtype0

Adamax/dense_23/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdamax/dense_23/bias/m
}
*Adamax/dense_23/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_23/bias/m*
_output_shapes
:*
dtype0

Adamax/dense_23/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@*)
shared_nameAdamax/dense_23/kernel/m

,Adamax/dense_23/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_23/kernel/m*
_output_shapes

:@*
dtype0

Adamax/dense_22/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_22/bias/m
}
*Adamax/dense_22/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_22/bias/m*
_output_shapes
:@*
dtype0

Adamax/dense_22/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_22/kernel/m

,Adamax/dense_22/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_22/kernel/m*
_output_shapes

:@@*
dtype0

Adamax/dense_21/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_21/bias/m
}
*Adamax/dense_21/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_21/bias/m*
_output_shapes
:@*
dtype0

Adamax/dense_21/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_21/kernel/m

,Adamax/dense_21/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_21/kernel/m*
_output_shapes

:@@*
dtype0

Adamax/dense_20/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_20/bias/m
}
*Adamax/dense_20/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_20/bias/m*
_output_shapes
:@*
dtype0

Adamax/dense_20/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_20/kernel/m

,Adamax/dense_20/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_20/kernel/m*
_output_shapes

:@@*
dtype0

Adamax/dense_19/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_19/bias/m
}
*Adamax/dense_19/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_19/bias/m*
_output_shapes
:@*
dtype0

Adamax/dense_19/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@*)
shared_nameAdamax/dense_19/kernel/m

,Adamax/dense_19/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_19/kernel/m*
_output_shapes

:@@*
dtype0

Adamax/dense_18/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*'
shared_nameAdamax/dense_18/bias/m
}
*Adamax/dense_18/bias/m/Read/ReadVariableOpReadVariableOpAdamax/dense_18/bias/m*
_output_shapes
:@*
dtype0

Adamax/dense_18/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@*)
shared_nameAdamax/dense_18/kernel/m

,Adamax/dense_18/kernel/m/Read/ReadVariableOpReadVariableOpAdamax/dense_18/kernel/m*
_output_shapes

:@*
dtype0
^
countVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_namecount
W
count/Read/ReadVariableOpReadVariableOpcount*
_output_shapes
: *
dtype0
^
totalVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nametotal
W
total/Read/ReadVariableOpReadVariableOptotal*
_output_shapes
: *
dtype0
b
count_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	count_1
[
count_1/Read/ReadVariableOpReadVariableOpcount_1*
_output_shapes
: *
dtype0
b
total_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	total_1
[
total_1/Read/ReadVariableOpReadVariableOptotal_1*
_output_shapes
: *
dtype0
|
Adamax/learning_rateVarHandleOp*
_output_shapes
: *
dtype0*
shape: *%
shared_nameAdamax/learning_rate
u
(Adamax/learning_rate/Read/ReadVariableOpReadVariableOpAdamax/learning_rate*
_output_shapes
: *
dtype0
l
Adamax/decayVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdamax/decay
e
 Adamax/decay/Read/ReadVariableOpReadVariableOpAdamax/decay*
_output_shapes
: *
dtype0
n
Adamax/beta_2VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdamax/beta_2
g
!Adamax/beta_2/Read/ReadVariableOpReadVariableOpAdamax/beta_2*
_output_shapes
: *
dtype0
n
Adamax/beta_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdamax/beta_1
g
!Adamax/beta_1/Read/ReadVariableOpReadVariableOpAdamax/beta_1*
_output_shapes
: *
dtype0
j
Adamax/iterVarHandleOp*
_output_shapes
: *
dtype0	*
shape: *
shared_nameAdamax/iter
c
Adamax/iter/Read/ReadVariableOpReadVariableOpAdamax/iter*
_output_shapes
: *
dtype0	
r
dense_23/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_namedense_23/bias
k
!dense_23/bias/Read/ReadVariableOpReadVariableOpdense_23/bias*
_output_shapes
:*
dtype0
z
dense_23/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@* 
shared_namedense_23/kernel
s
#dense_23/kernel/Read/ReadVariableOpReadVariableOpdense_23/kernel*
_output_shapes

:@*
dtype0
r
dense_22/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_namedense_22/bias
k
!dense_22/bias/Read/ReadVariableOpReadVariableOpdense_22/bias*
_output_shapes
:@*
dtype0
z
dense_22/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@* 
shared_namedense_22/kernel
s
#dense_22/kernel/Read/ReadVariableOpReadVariableOpdense_22/kernel*
_output_shapes

:@@*
dtype0
r
dense_21/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_namedense_21/bias
k
!dense_21/bias/Read/ReadVariableOpReadVariableOpdense_21/bias*
_output_shapes
:@*
dtype0
z
dense_21/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@* 
shared_namedense_21/kernel
s
#dense_21/kernel/Read/ReadVariableOpReadVariableOpdense_21/kernel*
_output_shapes

:@@*
dtype0
r
dense_20/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_namedense_20/bias
k
!dense_20/bias/Read/ReadVariableOpReadVariableOpdense_20/bias*
_output_shapes
:@*
dtype0
z
dense_20/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@* 
shared_namedense_20/kernel
s
#dense_20/kernel/Read/ReadVariableOpReadVariableOpdense_20/kernel*
_output_shapes

:@@*
dtype0
r
dense_19/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_namedense_19/bias
k
!dense_19/bias/Read/ReadVariableOpReadVariableOpdense_19/bias*
_output_shapes
:@*
dtype0
z
dense_19/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@@* 
shared_namedense_19/kernel
s
#dense_19/kernel/Read/ReadVariableOpReadVariableOpdense_19/kernel*
_output_shapes

:@@*
dtype0
r
dense_18/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_namedense_18/bias
k
!dense_18/bias/Read/ReadVariableOpReadVariableOpdense_18/bias*
_output_shapes
:@*
dtype0
z
dense_18/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:@* 
shared_namedense_18/kernel
s
#dense_18/kernel/Read/ReadVariableOpReadVariableOpdense_18/kernel*
_output_shapes

:@*
dtype0

NoOpNoOp
X
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*ËW
valueÁWB¾W B·W
Ã
layer_with_weights-0
layer-0
layer_with_weights-1
layer-1
layer_with_weights-2
layer-2
layer_with_weights-3
layer-3
layer_with_weights-4
layer-4
layer-5
layer_with_weights-5
layer-6
	variables
	trainable_variables

regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses
_default_save_signature
	optimizer

signatures*
¦
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses

kernel
bias*
¦
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses

kernel
 bias*
¦
!	variables
"trainable_variables
#regularization_losses
$	keras_api
%__call__
*&&call_and_return_all_conditional_losses

'kernel
(bias*
¦
)	variables
*trainable_variables
+regularization_losses
,	keras_api
-__call__
*.&call_and_return_all_conditional_losses

/kernel
0bias*
¦
1	variables
2trainable_variables
3regularization_losses
4	keras_api
5__call__
*6&call_and_return_all_conditional_losses

7kernel
8bias*
¥
9	variables
:trainable_variables
;regularization_losses
<	keras_api
=__call__
*>&call_and_return_all_conditional_losses
?_random_generator* 
¦
@	variables
Atrainable_variables
Bregularization_losses
C	keras_api
D__call__
*E&call_and_return_all_conditional_losses

Fkernel
Gbias*
Z
0
1
2
 3
'4
(5
/6
07
78
89
F10
G11*
Z
0
1
2
 3
'4
(5
/6
07
78
89
F10
G11*
H
H0
I1
J2
K3
L4
M5
N6
O7
P8
Q9* 
°
Rnon_trainable_variables

Slayers
Tmetrics
Ulayer_regularization_losses
Vlayer_metrics
	variables
	trainable_variables

regularization_losses
__call__
_default_save_signature
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses*
6
Wtrace_0
Xtrace_1
Ytrace_2
Ztrace_3* 
6
[trace_0
\trace_1
]trace_2
^trace_3* 
* 
´
_iter

`beta_1

abeta_2
	bdecay
clearning_ratem­m®m¯ m°'m±(m²/m³0m´7mµ8m¶Fm·Gm¸v¹vºv» v¼'v½(v¾/v¿0vÀ7vÁ8vÂFvÃGvÄ*

dserving_default* 

0
1*

0
1*

H0
I1* 

enon_trainable_variables

flayers
gmetrics
hlayer_regularization_losses
ilayer_metrics
	variables
trainable_variables
regularization_losses
__call__
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses*

jtrace_0* 

ktrace_0* 
_Y
VARIABLE_VALUEdense_18/kernel6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_18/bias4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUE*

0
 1*

0
 1*

J0
K1* 

lnon_trainable_variables

mlayers
nmetrics
olayer_regularization_losses
player_metrics
	variables
trainable_variables
regularization_losses
__call__
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses*

qtrace_0* 

rtrace_0* 
_Y
VARIABLE_VALUEdense_19/kernel6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_19/bias4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUE*

'0
(1*

'0
(1*

L0
M1* 

snon_trainable_variables

tlayers
umetrics
vlayer_regularization_losses
wlayer_metrics
!	variables
"trainable_variables
#regularization_losses
%__call__
*&&call_and_return_all_conditional_losses
&&"call_and_return_conditional_losses*

xtrace_0* 

ytrace_0* 
_Y
VARIABLE_VALUEdense_20/kernel6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_20/bias4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUE*

/0
01*

/0
01*

N0
O1* 

znon_trainable_variables

{layers
|metrics
}layer_regularization_losses
~layer_metrics
)	variables
*trainable_variables
+regularization_losses
-__call__
*.&call_and_return_all_conditional_losses
&."call_and_return_conditional_losses*

trace_0* 

trace_0* 
_Y
VARIABLE_VALUEdense_21/kernel6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_21/bias4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUE*

70
81*

70
81*

P0
Q1* 

non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
1	variables
2trainable_variables
3regularization_losses
5__call__
*6&call_and_return_all_conditional_losses
&6"call_and_return_conditional_losses*

trace_0* 

trace_0* 
_Y
VARIABLE_VALUEdense_22/kernel6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_22/bias4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUE*
* 
* 
* 

non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
9	variables
:trainable_variables
;regularization_losses
=__call__
*>&call_and_return_all_conditional_losses
&>"call_and_return_conditional_losses* 

trace_0
trace_1* 

trace_0
trace_1* 
* 

F0
G1*

F0
G1*
* 

non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
@	variables
Atrainable_variables
Bregularization_losses
D__call__
*E&call_and_return_all_conditional_losses
&E"call_and_return_conditional_losses*

trace_0* 

trace_0* 
_Y
VARIABLE_VALUEdense_23/kernel6layer_with_weights-5/kernel/.ATTRIBUTES/VARIABLE_VALUE*
[U
VARIABLE_VALUEdense_23/bias4layer_with_weights-5/bias/.ATTRIBUTES/VARIABLE_VALUE*

trace_0* 

trace_0* 

trace_0* 

trace_0* 

trace_0* 

trace_0* 

trace_0* 

trace_0* 

 trace_0* 

¡trace_0* 
* 
5
0
1
2
3
4
5
6*

¢0
£1*
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
NH
VARIABLE_VALUEAdamax/iter)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUE*
RL
VARIABLE_VALUEAdamax/beta_1+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUE*
RL
VARIABLE_VALUEAdamax/beta_2+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUE*
PJ
VARIABLE_VALUEAdamax/decay*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUE*
`Z
VARIABLE_VALUEAdamax/learning_rate2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUE*
* 
* 
* 
* 

H0
I1* 
* 
* 
* 
* 
* 
* 

J0
K1* 
* 
* 
* 
* 
* 
* 

L0
M1* 
* 
* 
* 
* 
* 
* 

N0
O1* 
* 
* 
* 
* 
* 
* 

P0
Q1* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
<
¤	variables
¥	keras_api

¦total

§count*
M
¨	variables
©	keras_api

ªtotal

«count
¬
_fn_kwargs*

¦0
§1*

¤	variables*
UO
VARIABLE_VALUEtotal_14keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUE*
UO
VARIABLE_VALUEcount_14keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUE*

ª0
«1*

¨	variables*
SM
VARIABLE_VALUEtotal4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUE*
SM
VARIABLE_VALUEcount4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUE*
* 
~
VARIABLE_VALUEAdamax/dense_18/kernel/mRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_18/bias/mPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_19/kernel/mRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_19/bias/mPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_20/kernel/mRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_20/bias/mPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_21/kernel/mRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_21/bias/mPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_22/kernel/mRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_22/bias/mPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_23/kernel/mRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_23/bias/mPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_18/kernel/vRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_18/bias/vPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_19/kernel/vRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_19/bias/vPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_20/kernel/vRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_20/bias/vPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_21/kernel/vRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_21/bias/vPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_22/kernel/vRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_22/bias/vPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
~
VARIABLE_VALUEAdamax/dense_23/kernel/vRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
VARIABLE_VALUEAdamax/dense_23/bias/vPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE*
z
serving_default_input_4Placeholder*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*
dtype0*
shape:ÿÿÿÿÿÿÿÿÿ

StatefulPartitionedCallStatefulPartitionedCallserving_default_input_4dense_18/kerneldense_18/biasdense_19/kerneldense_19/biasdense_20/kerneldense_20/biasdense_21/kerneldense_21/biasdense_22/kerneldense_22/biasdense_23/kerneldense_23/bias*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *.
f)R'
%__inference_signature_wrapper_2748360
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
Ô
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename#dense_18/kernel/Read/ReadVariableOp!dense_18/bias/Read/ReadVariableOp#dense_19/kernel/Read/ReadVariableOp!dense_19/bias/Read/ReadVariableOp#dense_20/kernel/Read/ReadVariableOp!dense_20/bias/Read/ReadVariableOp#dense_21/kernel/Read/ReadVariableOp!dense_21/bias/Read/ReadVariableOp#dense_22/kernel/Read/ReadVariableOp!dense_22/bias/Read/ReadVariableOp#dense_23/kernel/Read/ReadVariableOp!dense_23/bias/Read/ReadVariableOpAdamax/iter/Read/ReadVariableOp!Adamax/beta_1/Read/ReadVariableOp!Adamax/beta_2/Read/ReadVariableOp Adamax/decay/Read/ReadVariableOp(Adamax/learning_rate/Read/ReadVariableOptotal_1/Read/ReadVariableOpcount_1/Read/ReadVariableOptotal/Read/ReadVariableOpcount/Read/ReadVariableOp,Adamax/dense_18/kernel/m/Read/ReadVariableOp*Adamax/dense_18/bias/m/Read/ReadVariableOp,Adamax/dense_19/kernel/m/Read/ReadVariableOp*Adamax/dense_19/bias/m/Read/ReadVariableOp,Adamax/dense_20/kernel/m/Read/ReadVariableOp*Adamax/dense_20/bias/m/Read/ReadVariableOp,Adamax/dense_21/kernel/m/Read/ReadVariableOp*Adamax/dense_21/bias/m/Read/ReadVariableOp,Adamax/dense_22/kernel/m/Read/ReadVariableOp*Adamax/dense_22/bias/m/Read/ReadVariableOp,Adamax/dense_23/kernel/m/Read/ReadVariableOp*Adamax/dense_23/bias/m/Read/ReadVariableOp,Adamax/dense_18/kernel/v/Read/ReadVariableOp*Adamax/dense_18/bias/v/Read/ReadVariableOp,Adamax/dense_19/kernel/v/Read/ReadVariableOp*Adamax/dense_19/bias/v/Read/ReadVariableOp,Adamax/dense_20/kernel/v/Read/ReadVariableOp*Adamax/dense_20/bias/v/Read/ReadVariableOp,Adamax/dense_21/kernel/v/Read/ReadVariableOp*Adamax/dense_21/bias/v/Read/ReadVariableOp,Adamax/dense_22/kernel/v/Read/ReadVariableOp*Adamax/dense_22/bias/v/Read/ReadVariableOp,Adamax/dense_23/kernel/v/Read/ReadVariableOp*Adamax/dense_23/bias/v/Read/ReadVariableOpConst*:
Tin3
12/	*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *)
f$R"
 __inference__traced_save_2749171
Ë	
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenamedense_18/kerneldense_18/biasdense_19/kerneldense_19/biasdense_20/kerneldense_20/biasdense_21/kerneldense_21/biasdense_22/kerneldense_22/biasdense_23/kerneldense_23/biasAdamax/iterAdamax/beta_1Adamax/beta_2Adamax/decayAdamax/learning_ratetotal_1count_1totalcountAdamax/dense_18/kernel/mAdamax/dense_18/bias/mAdamax/dense_19/kernel/mAdamax/dense_19/bias/mAdamax/dense_20/kernel/mAdamax/dense_20/bias/mAdamax/dense_21/kernel/mAdamax/dense_21/bias/mAdamax/dense_22/kernel/mAdamax/dense_22/bias/mAdamax/dense_23/kernel/mAdamax/dense_23/bias/mAdamax/dense_18/kernel/vAdamax/dense_18/bias/vAdamax/dense_19/kernel/vAdamax/dense_19/bias/vAdamax/dense_20/kernel/vAdamax/dense_20/bias/vAdamax/dense_21/kernel/vAdamax/dense_21/bias/vAdamax/dense_22/kernel/vAdamax/dense_22/bias/vAdamax/dense_23/kernel/vAdamax/dense_23/bias/v*9
Tin2
02.*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *,
f'R%
#__inference__traced_restore_2749316À

ª
__inference_loss_fn_3_2748947F
8dense_19_bias_regularizer_square_readvariableop_resource:@
identity¢/dense_19/bias/Regularizer/Square/ReadVariableOp¤
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOp8dense_19_bias_regularizer_square_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: _
IdentityIdentity!dense_19/bias/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: x
NoOpNoOp0^dense_19/bias/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp

ª
__inference_loss_fn_1_2748925F
8dense_18_bias_regularizer_square_readvariableop_resource:@
identity¢/dense_18/bias/Regularizer/Square/ReadVariableOp¤
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOp8dense_18_bias_regularizer_square_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: _
IdentityIdentity!dense_18/bias/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: x
NoOpNoOp0^dense_18/bias/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp
¸
²
__inference_loss_fn_6_2748980L
:dense_21_kernel_regularizer_square_readvariableop_resource:@@
identity¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¬
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOp:dense_21_kernel_regularizer_square_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: a
IdentityIdentity#dense_21/kernel/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: z
NoOpNoOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp
ô	
e
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747838

inputs
identityR
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *ä8?d
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@C
dropout/ShapeShapeinputs*
T0*
_output_shapes
:
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*
dtype0[
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *ÍÌÌ=¦
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@o
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@i
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Y
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs

ª
__inference_loss_fn_5_2748969F
8dense_20_bias_regularizer_square_readvariableop_resource:@
identity¢/dense_20/bias/Regularizer/Square/ReadVariableOp¤
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOp8dense_20_bias_regularizer_square_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: _
IdentityIdentity!dense_20/bias/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: x
NoOpNoOp0^dense_20/bias/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp
¯
ª
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748697

inputs9
'dense_18_matmul_readvariableop_resource:@6
(dense_18_biasadd_readvariableop_resource:@9
'dense_19_matmul_readvariableop_resource:@@6
(dense_19_biasadd_readvariableop_resource:@9
'dense_20_matmul_readvariableop_resource:@@6
(dense_20_biasadd_readvariableop_resource:@9
'dense_21_matmul_readvariableop_resource:@@6
(dense_21_biasadd_readvariableop_resource:@9
'dense_22_matmul_readvariableop_resource:@@6
(dense_22_biasadd_readvariableop_resource:@9
'dense_23_matmul_readvariableop_resource:@6
(dense_23_biasadd_readvariableop_resource:
identity¢dense_18/BiasAdd/ReadVariableOp¢dense_18/MatMul/ReadVariableOp¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢dense_19/BiasAdd/ReadVariableOp¢dense_19/MatMul/ReadVariableOp¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢dense_20/BiasAdd/ReadVariableOp¢dense_20/MatMul/ReadVariableOp¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢dense_21/BiasAdd/ReadVariableOp¢dense_21/MatMul/ReadVariableOp¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢dense_22/BiasAdd/ReadVariableOp¢dense_22/MatMul/ReadVariableOp¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢dense_23/BiasAdd/ReadVariableOp¢dense_23/MatMul/ReadVariableOp
dense_18/MatMul/ReadVariableOpReadVariableOp'dense_18_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0{
dense_18/MatMulMatMulinputs&dense_18/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_18/BiasAdd/ReadVariableOpReadVariableOp(dense_18_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_18/BiasAddBiasAdddense_18/MatMul:product:0'dense_18/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_18/TanhTanhdense_18/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_19/MatMul/ReadVariableOpReadVariableOp'dense_19_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_19/MatMulMatMuldense_18/Tanh:y:0&dense_19/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_19/BiasAdd/ReadVariableOpReadVariableOp(dense_19_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_19/BiasAddBiasAdddense_19/MatMul:product:0'dense_19/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_19/TanhTanhdense_19/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_20/MatMul/ReadVariableOpReadVariableOp'dense_20_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_20/MatMulMatMuldense_19/Tanh:y:0&dense_20/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_20/BiasAdd/ReadVariableOpReadVariableOp(dense_20_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_20/BiasAddBiasAdddense_20/MatMul:product:0'dense_20/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_20/TanhTanhdense_20/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_21/MatMul/ReadVariableOpReadVariableOp'dense_21_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_21/MatMulMatMuldense_20/Tanh:y:0&dense_21/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_21/BiasAdd/ReadVariableOpReadVariableOp(dense_21_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_21/BiasAddBiasAdddense_21/MatMul:product:0'dense_21/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_21/TanhTanhdense_21/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_22/MatMul/ReadVariableOpReadVariableOp'dense_22_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_22/MatMulMatMuldense_21/Tanh:y:0&dense_22/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_22/BiasAdd/ReadVariableOpReadVariableOp(dense_22_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_22/BiasAddBiasAdddense_22/MatMul:product:0'dense_22/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_22/TanhTanhdense_22/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@\
dropout_3/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *ä8?
dropout_3/dropout/MulMuldense_22/Tanh:y:0 dropout_3/dropout/Const:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@X
dropout_3/dropout/ShapeShapedense_22/Tanh:y:0*
T0*
_output_shapes
: 
.dropout_3/dropout/random_uniform/RandomUniformRandomUniform dropout_3/dropout/Shape:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*
dtype0e
 dropout_3/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *ÍÌÌ=Ä
dropout_3/dropout/GreaterEqualGreaterEqual7dropout_3/dropout/random_uniform/RandomUniform:output:0)dropout_3/dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dropout_3/dropout/CastCast"dropout_3/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dropout_3/dropout/Mul_1Muldropout_3/dropout/Mul:z:0dropout_3/dropout/Cast:y:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_23/MatMul/ReadVariableOpReadVariableOp'dense_23_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0
dense_23/MatMulMatMuldropout_3/dropout/Mul_1:z:0&dense_23/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
dense_23/BiasAdd/ReadVariableOpReadVariableOp(dense_23_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0
dense_23/BiasAddBiasAdddense_23/MatMul:product:0'dense_23/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_18_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_18_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_19_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_19_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_20_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_20_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_21_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_21_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_22_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_22_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: h
IdentityIdentitydense_23/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿÖ
NoOpNoOp ^dense_18/BiasAdd/ReadVariableOp^dense_18/MatMul/ReadVariableOp0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp ^dense_19/BiasAdd/ReadVariableOp^dense_19/MatMul/ReadVariableOp0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp ^dense_20/BiasAdd/ReadVariableOp^dense_20/MatMul/ReadVariableOp0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp ^dense_21/BiasAdd/ReadVariableOp^dense_21/MatMul/ReadVariableOp0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp ^dense_22/BiasAdd/ReadVariableOp^dense_22/MatMul/ReadVariableOp0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp ^dense_23/BiasAdd/ReadVariableOp^dense_23/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2B
dense_18/BiasAdd/ReadVariableOpdense_18/BiasAdd/ReadVariableOp2@
dense_18/MatMul/ReadVariableOpdense_18/MatMul/ReadVariableOp2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2B
dense_19/BiasAdd/ReadVariableOpdense_19/BiasAdd/ReadVariableOp2@
dense_19/MatMul/ReadVariableOpdense_19/MatMul/ReadVariableOp2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2B
dense_20/BiasAdd/ReadVariableOpdense_20/BiasAdd/ReadVariableOp2@
dense_20/MatMul/ReadVariableOpdense_20/MatMul/ReadVariableOp2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2B
dense_21/BiasAdd/ReadVariableOpdense_21/BiasAdd/ReadVariableOp2@
dense_21/MatMul/ReadVariableOpdense_21/MatMul/ReadVariableOp2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2B
dense_22/BiasAdd/ReadVariableOpdense_22/BiasAdd/ReadVariableOp2@
dense_22/MatMul/ReadVariableOpdense_22/MatMul/ReadVariableOp2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2B
dense_23/BiasAdd/ReadVariableOpdense_23/BiasAdd/ReadVariableOp2@
dense_23/MatMul/ReadVariableOpdense_23/MatMul/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
ò

ª
.__inference_sequential_3_layer_call_fn_2748478

inputs
unknown:@
	unknown_0:@
	unknown_1:@@
	unknown_2:@
	unknown_3:@@
	unknown_4:@
	unknown_5:@@
	unknown_6:@
	unknown_7:@@
	unknown_8:@
	unknown_9:@

unknown_10:
identity¢StatefulPartitionedCallá
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *R
fMRK
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748017o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
ò

ª
.__inference_sequential_3_layer_call_fn_2748449

inputs
unknown:@
	unknown_0:@
	unknown_1:@@
	unknown_2:@
	unknown_3:@@
	unknown_4:@
	unknown_5:@@
	unknown_6:@
	unknown_7:@@
	unknown_8:@
	unknown_9:@

unknown_10:
identity¢StatefulPartitionedCallá
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *R
fMRK
I__inference_sequential_3_layer_call_and_return_conditional_losses_2747781o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
èA
¾
"__inference__wrapped_model_2747545
input_4F
4sequential_3_dense_18_matmul_readvariableop_resource:@C
5sequential_3_dense_18_biasadd_readvariableop_resource:@F
4sequential_3_dense_19_matmul_readvariableop_resource:@@C
5sequential_3_dense_19_biasadd_readvariableop_resource:@F
4sequential_3_dense_20_matmul_readvariableop_resource:@@C
5sequential_3_dense_20_biasadd_readvariableop_resource:@F
4sequential_3_dense_21_matmul_readvariableop_resource:@@C
5sequential_3_dense_21_biasadd_readvariableop_resource:@F
4sequential_3_dense_22_matmul_readvariableop_resource:@@C
5sequential_3_dense_22_biasadd_readvariableop_resource:@F
4sequential_3_dense_23_matmul_readvariableop_resource:@C
5sequential_3_dense_23_biasadd_readvariableop_resource:
identity¢,sequential_3/dense_18/BiasAdd/ReadVariableOp¢+sequential_3/dense_18/MatMul/ReadVariableOp¢,sequential_3/dense_19/BiasAdd/ReadVariableOp¢+sequential_3/dense_19/MatMul/ReadVariableOp¢,sequential_3/dense_20/BiasAdd/ReadVariableOp¢+sequential_3/dense_20/MatMul/ReadVariableOp¢,sequential_3/dense_21/BiasAdd/ReadVariableOp¢+sequential_3/dense_21/MatMul/ReadVariableOp¢,sequential_3/dense_22/BiasAdd/ReadVariableOp¢+sequential_3/dense_22/MatMul/ReadVariableOp¢,sequential_3/dense_23/BiasAdd/ReadVariableOp¢+sequential_3/dense_23/MatMul/ReadVariableOp 
+sequential_3/dense_18/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_18_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0
sequential_3/dense_18/MatMulMatMulinput_43sequential_3/dense_18/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
,sequential_3/dense_18/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_18_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0¸
sequential_3/dense_18/BiasAddBiasAdd&sequential_3/dense_18/MatMul:product:04sequential_3/dense_18/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@|
sequential_3/dense_18/TanhTanh&sequential_3/dense_18/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@ 
+sequential_3/dense_19/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_19_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0­
sequential_3/dense_19/MatMulMatMulsequential_3/dense_18/Tanh:y:03sequential_3/dense_19/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
,sequential_3/dense_19/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_19_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0¸
sequential_3/dense_19/BiasAddBiasAdd&sequential_3/dense_19/MatMul:product:04sequential_3/dense_19/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@|
sequential_3/dense_19/TanhTanh&sequential_3/dense_19/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@ 
+sequential_3/dense_20/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_20_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0­
sequential_3/dense_20/MatMulMatMulsequential_3/dense_19/Tanh:y:03sequential_3/dense_20/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
,sequential_3/dense_20/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_20_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0¸
sequential_3/dense_20/BiasAddBiasAdd&sequential_3/dense_20/MatMul:product:04sequential_3/dense_20/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@|
sequential_3/dense_20/TanhTanh&sequential_3/dense_20/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@ 
+sequential_3/dense_21/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_21_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0­
sequential_3/dense_21/MatMulMatMulsequential_3/dense_20/Tanh:y:03sequential_3/dense_21/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
,sequential_3/dense_21/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_21_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0¸
sequential_3/dense_21/BiasAddBiasAdd&sequential_3/dense_21/MatMul:product:04sequential_3/dense_21/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@|
sequential_3/dense_21/TanhTanh&sequential_3/dense_21/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@ 
+sequential_3/dense_22/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_22_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0­
sequential_3/dense_22/MatMulMatMulsequential_3/dense_21/Tanh:y:03sequential_3/dense_22/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
,sequential_3/dense_22/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_22_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0¸
sequential_3/dense_22/BiasAddBiasAdd&sequential_3/dense_22/MatMul:product:04sequential_3/dense_22/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@|
sequential_3/dense_22/TanhTanh&sequential_3/dense_22/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@}
sequential_3/dropout_3/IdentityIdentitysequential_3/dense_22/Tanh:y:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@ 
+sequential_3/dense_23/MatMul/ReadVariableOpReadVariableOp4sequential_3_dense_23_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0·
sequential_3/dense_23/MatMulMatMul(sequential_3/dropout_3/Identity:output:03sequential_3/dense_23/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
,sequential_3/dense_23/BiasAdd/ReadVariableOpReadVariableOp5sequential_3_dense_23_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0¸
sequential_3/dense_23/BiasAddBiasAdd&sequential_3/dense_23/MatMul:product:04sequential_3/dense_23/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿu
IdentityIdentity&sequential_3/dense_23/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿô
NoOpNoOp-^sequential_3/dense_18/BiasAdd/ReadVariableOp,^sequential_3/dense_18/MatMul/ReadVariableOp-^sequential_3/dense_19/BiasAdd/ReadVariableOp,^sequential_3/dense_19/MatMul/ReadVariableOp-^sequential_3/dense_20/BiasAdd/ReadVariableOp,^sequential_3/dense_20/MatMul/ReadVariableOp-^sequential_3/dense_21/BiasAdd/ReadVariableOp,^sequential_3/dense_21/MatMul/ReadVariableOp-^sequential_3/dense_22/BiasAdd/ReadVariableOp,^sequential_3/dense_22/MatMul/ReadVariableOp-^sequential_3/dense_23/BiasAdd/ReadVariableOp,^sequential_3/dense_23/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2\
,sequential_3/dense_18/BiasAdd/ReadVariableOp,sequential_3/dense_18/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_18/MatMul/ReadVariableOp+sequential_3/dense_18/MatMul/ReadVariableOp2\
,sequential_3/dense_19/BiasAdd/ReadVariableOp,sequential_3/dense_19/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_19/MatMul/ReadVariableOp+sequential_3/dense_19/MatMul/ReadVariableOp2\
,sequential_3/dense_20/BiasAdd/ReadVariableOp,sequential_3/dense_20/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_20/MatMul/ReadVariableOp+sequential_3/dense_20/MatMul/ReadVariableOp2\
,sequential_3/dense_21/BiasAdd/ReadVariableOp,sequential_3/dense_21/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_21/MatMul/ReadVariableOp+sequential_3/dense_21/MatMul/ReadVariableOp2\
,sequential_3/dense_22/BiasAdd/ReadVariableOp,sequential_3/dense_22/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_22/MatMul/ReadVariableOp+sequential_3/dense_22/MatMul/ReadVariableOp2\
,sequential_3/dense_23/BiasAdd/ReadVariableOp,sequential_3/dense_23/BiasAdd/ReadVariableOp2Z
+sequential_3/dense_23/MatMul/ReadVariableOp+sequential_3/dense_23/MatMul/ReadVariableOp:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4
È	
ö
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714

inputs0
matmul_readvariableop_resource:@-
biasadd_readvariableop_resource:
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿr
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ_
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿw
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ä

*__inference_dense_23_layer_call_fn_2748893

inputs
unknown:@
	unknown_0:
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ä

*__inference_dense_22_layer_call_fn_2748834

inputs
unknown:@@
	unknown_0:@
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ù
d
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747702

inputs

identity_1N
IdentityIdentityinputs*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@[

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
õ

«
.__inference_sequential_3_layer_call_fn_2748073
input_4
unknown:@
	unknown_0:@
	unknown_1:@@
	unknown_2:@
	unknown_3:@@
	unknown_4:@
	unknown_5:@@
	unknown_6:@
	unknown_7:@@
	unknown_8:@
	unknown_9:@

unknown_10:
identity¢StatefulPartitionedCallâ
StatefulPartitionedCallStatefulPartitionedCallinput_4unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *R
fMRK
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748017o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4
æq
Ñ	
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748168
input_4"
dense_18_2748076:@
dense_18_2748078:@"
dense_19_2748081:@@
dense_19_2748083:@"
dense_20_2748086:@@
dense_20_2748088:@"
dense_21_2748091:@@
dense_21_2748093:@"
dense_22_2748096:@@
dense_22_2748098:@"
dense_23_2748102:@
dense_23_2748104:
identity¢ dense_18/StatefulPartitionedCall¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢ dense_19/StatefulPartitionedCall¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢ dense_20/StatefulPartitionedCall¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢ dense_21/StatefulPartitionedCall¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢ dense_22/StatefulPartitionedCall¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢ dense_23/StatefulPartitionedCallô
 dense_18/StatefulPartitionedCallStatefulPartitionedCallinput_4dense_18_2748076dense_18_2748078*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575
 dense_19/StatefulPartitionedCallStatefulPartitionedCall)dense_18/StatefulPartitionedCall:output:0dense_19_2748081dense_19_2748083*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604
 dense_20/StatefulPartitionedCallStatefulPartitionedCall)dense_19/StatefulPartitionedCall:output:0dense_20_2748086dense_20_2748088*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633
 dense_21/StatefulPartitionedCallStatefulPartitionedCall)dense_20/StatefulPartitionedCall:output:0dense_21_2748091dense_21_2748093*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662
 dense_22/StatefulPartitionedCallStatefulPartitionedCall)dense_21/StatefulPartitionedCall:output:0dense_22_2748096dense_22_2748098*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691Þ
dropout_3/PartitionedCallPartitionedCall)dense_22/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747702
 dense_23/StatefulPartitionedCallStatefulPartitionedCall"dropout_3/PartitionedCall:output:0dense_23_2748102dense_23_2748104*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2748076*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2748078*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2748081*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2748083*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2748086*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2748088*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2748091*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2748093*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2748096*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2748098*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: x
IdentityIdentity)dense_23/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
NoOpNoOp!^dense_18/StatefulPartitionedCall0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp!^dense_19/StatefulPartitionedCall0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp!^dense_20/StatefulPartitionedCall0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp!^dense_21/StatefulPartitionedCall0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp!^dense_22/StatefulPartitionedCall0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp!^dense_23/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2D
 dense_18/StatefulPartitionedCall dense_18/StatefulPartitionedCall2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2D
 dense_19/StatefulPartitionedCall dense_19/StatefulPartitionedCall2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2D
 dense_20/StatefulPartitionedCall dense_20/StatefulPartitionedCall2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2D
 dense_21/StatefulPartitionedCall dense_21/StatefulPartitionedCall2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2D
 dense_22/StatefulPartitionedCall dense_22/StatefulPartitionedCall2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2D
 dense_23/StatefulPartitionedCall dense_23/StatefulPartitionedCall:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4

Ü
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs

Ü
E__inference_dense_20_layer_call_and_return_conditional_losses_2748793

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs

Ü
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575

inputs0
matmul_readvariableop_resource:@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
Ù
d
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748872

inputs

identity_1N
IdentityIdentityinputs*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@[

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
s
õ	
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748263
input_4"
dense_18_2748171:@
dense_18_2748173:@"
dense_19_2748176:@@
dense_19_2748178:@"
dense_20_2748181:@@
dense_20_2748183:@"
dense_21_2748186:@@
dense_21_2748188:@"
dense_22_2748191:@@
dense_22_2748193:@"
dense_23_2748197:@
dense_23_2748199:
identity¢ dense_18/StatefulPartitionedCall¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢ dense_19/StatefulPartitionedCall¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢ dense_20/StatefulPartitionedCall¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢ dense_21/StatefulPartitionedCall¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢ dense_22/StatefulPartitionedCall¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢ dense_23/StatefulPartitionedCall¢!dropout_3/StatefulPartitionedCallô
 dense_18/StatefulPartitionedCallStatefulPartitionedCallinput_4dense_18_2748171dense_18_2748173*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575
 dense_19/StatefulPartitionedCallStatefulPartitionedCall)dense_18/StatefulPartitionedCall:output:0dense_19_2748176dense_19_2748178*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604
 dense_20/StatefulPartitionedCallStatefulPartitionedCall)dense_19/StatefulPartitionedCall:output:0dense_20_2748181dense_20_2748183*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633
 dense_21/StatefulPartitionedCallStatefulPartitionedCall)dense_20/StatefulPartitionedCall:output:0dense_21_2748186dense_21_2748188*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662
 dense_22/StatefulPartitionedCallStatefulPartitionedCall)dense_21/StatefulPartitionedCall:output:0dense_22_2748191dense_22_2748193*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691î
!dropout_3/StatefulPartitionedCallStatefulPartitionedCall)dense_22/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747838
 dense_23/StatefulPartitionedCallStatefulPartitionedCall*dropout_3/StatefulPartitionedCall:output:0dense_23_2748197dense_23_2748199*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2748171*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2748173*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2748176*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2748178*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2748181*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2748183*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2748186*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2748188*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2748191*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2748193*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: x
IdentityIdentity)dense_23/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿº
NoOpNoOp!^dense_18/StatefulPartitionedCall0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp!^dense_19/StatefulPartitionedCall0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp!^dense_20/StatefulPartitionedCall0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp!^dense_21/StatefulPartitionedCall0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp!^dense_22/StatefulPartitionedCall0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp!^dense_23/StatefulPartitionedCall"^dropout_3/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2D
 dense_18/StatefulPartitionedCall dense_18/StatefulPartitionedCall2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2D
 dense_19/StatefulPartitionedCall dense_19/StatefulPartitionedCall2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2D
 dense_20/StatefulPartitionedCall dense_20/StatefulPartitionedCall2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2D
 dense_21/StatefulPartitionedCall dense_21/StatefulPartitionedCall2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2D
 dense_22/StatefulPartitionedCall dense_22/StatefulPartitionedCall2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2D
 dense_23/StatefulPartitionedCall dense_23/StatefulPartitionedCall2F
!dropout_3/StatefulPartitionedCall!dropout_3/StatefulPartitionedCall:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4
¸
²
__inference_loss_fn_2_2748936L
:dense_19_kernel_regularizer_square_readvariableop_resource:@@
identity¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¬
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOp:dense_19_kernel_regularizer_square_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: a
IdentityIdentity#dense_19/kernel/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: z
NoOpNoOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp
ó
d
+__inference_dropout_3_layer_call_fn_2748867

inputs
identity¢StatefulPartitionedCallÁ
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747838o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
¸
²
__inference_loss_fn_8_2749002L
:dense_22_kernel_regularizer_square_readvariableop_resource:@@
identity¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¬
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOp:dense_22_kernel_regularizer_square_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: a
IdentityIdentity#dense_22/kernel/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: z
NoOpNoOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp
ú³
µ
#__inference__traced_restore_2749316
file_prefix2
 assignvariableop_dense_18_kernel:@.
 assignvariableop_1_dense_18_bias:@4
"assignvariableop_2_dense_19_kernel:@@.
 assignvariableop_3_dense_19_bias:@4
"assignvariableop_4_dense_20_kernel:@@.
 assignvariableop_5_dense_20_bias:@4
"assignvariableop_6_dense_21_kernel:@@.
 assignvariableop_7_dense_21_bias:@4
"assignvariableop_8_dense_22_kernel:@@.
 assignvariableop_9_dense_22_bias:@5
#assignvariableop_10_dense_23_kernel:@/
!assignvariableop_11_dense_23_bias:)
assignvariableop_12_adamax_iter:	 +
!assignvariableop_13_adamax_beta_1: +
!assignvariableop_14_adamax_beta_2: *
 assignvariableop_15_adamax_decay: 2
(assignvariableop_16_adamax_learning_rate: %
assignvariableop_17_total_1: %
assignvariableop_18_count_1: #
assignvariableop_19_total: #
assignvariableop_20_count: >
,assignvariableop_21_adamax_dense_18_kernel_m:@8
*assignvariableop_22_adamax_dense_18_bias_m:@>
,assignvariableop_23_adamax_dense_19_kernel_m:@@8
*assignvariableop_24_adamax_dense_19_bias_m:@>
,assignvariableop_25_adamax_dense_20_kernel_m:@@8
*assignvariableop_26_adamax_dense_20_bias_m:@>
,assignvariableop_27_adamax_dense_21_kernel_m:@@8
*assignvariableop_28_adamax_dense_21_bias_m:@>
,assignvariableop_29_adamax_dense_22_kernel_m:@@8
*assignvariableop_30_adamax_dense_22_bias_m:@>
,assignvariableop_31_adamax_dense_23_kernel_m:@8
*assignvariableop_32_adamax_dense_23_bias_m:>
,assignvariableop_33_adamax_dense_18_kernel_v:@8
*assignvariableop_34_adamax_dense_18_bias_v:@>
,assignvariableop_35_adamax_dense_19_kernel_v:@@8
*assignvariableop_36_adamax_dense_19_bias_v:@>
,assignvariableop_37_adamax_dense_20_kernel_v:@@8
*assignvariableop_38_adamax_dense_20_bias_v:@>
,assignvariableop_39_adamax_dense_21_kernel_v:@@8
*assignvariableop_40_adamax_dense_21_bias_v:@>
,assignvariableop_41_adamax_dense_22_kernel_v:@@8
*assignvariableop_42_adamax_dense_22_bias_v:@>
,assignvariableop_43_adamax_dense_23_kernel_v:@8
*assignvariableop_44_adamax_dense_23_bias_v:
identity_46¢AssignVariableOp¢AssignVariableOp_1¢AssignVariableOp_10¢AssignVariableOp_11¢AssignVariableOp_12¢AssignVariableOp_13¢AssignVariableOp_14¢AssignVariableOp_15¢AssignVariableOp_16¢AssignVariableOp_17¢AssignVariableOp_18¢AssignVariableOp_19¢AssignVariableOp_2¢AssignVariableOp_20¢AssignVariableOp_21¢AssignVariableOp_22¢AssignVariableOp_23¢AssignVariableOp_24¢AssignVariableOp_25¢AssignVariableOp_26¢AssignVariableOp_27¢AssignVariableOp_28¢AssignVariableOp_29¢AssignVariableOp_3¢AssignVariableOp_30¢AssignVariableOp_31¢AssignVariableOp_32¢AssignVariableOp_33¢AssignVariableOp_34¢AssignVariableOp_35¢AssignVariableOp_36¢AssignVariableOp_37¢AssignVariableOp_38¢AssignVariableOp_39¢AssignVariableOp_4¢AssignVariableOp_40¢AssignVariableOp_41¢AssignVariableOp_42¢AssignVariableOp_43¢AssignVariableOp_44¢AssignVariableOp_5¢AssignVariableOp_6¢AssignVariableOp_7¢AssignVariableOp_8¢AssignVariableOp_9¦
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:.*
dtype0*Ì
valueÂB¿.B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-5/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-5/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPHÌ
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:.*
dtype0*o
valuefBd.B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*Î
_output_shapes»
¸::::::::::::::::::::::::::::::::::::::::::::::*<
dtypes2
02.	[
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOpAssignVariableOp assignvariableop_dense_18_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_1AssignVariableOp assignvariableop_1_dense_18_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_2AssignVariableOp"assignvariableop_2_dense_19_kernelIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_3AssignVariableOp assignvariableop_3_dense_19_biasIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_4AssignVariableOp"assignvariableop_4_dense_20_kernelIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_5AssignVariableOp assignvariableop_5_dense_20_biasIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_6AssignVariableOp"assignvariableop_6_dense_21_kernelIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_7AssignVariableOp assignvariableop_7_dense_21_biasIdentity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_8AssignVariableOp"assignvariableop_8_dense_22_kernelIdentity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_9AssignVariableOp assignvariableop_9_dense_22_biasIdentity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_10AssignVariableOp#assignvariableop_10_dense_23_kernelIdentity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_11AssignVariableOp!assignvariableop_11_dense_23_biasIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0	*
_output_shapes
:
AssignVariableOp_12AssignVariableOpassignvariableop_12_adamax_iterIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype0	_
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_13AssignVariableOp!assignvariableop_13_adamax_beta_1Identity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_14AssignVariableOp!assignvariableop_14_adamax_beta_2Identity_14:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_15AssignVariableOp assignvariableop_15_adamax_decayIdentity_15:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_16IdentityRestoreV2:tensors:16"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_16AssignVariableOp(assignvariableop_16_adamax_learning_rateIdentity_16:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_17IdentityRestoreV2:tensors:17"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_17AssignVariableOpassignvariableop_17_total_1Identity_17:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_18IdentityRestoreV2:tensors:18"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_18AssignVariableOpassignvariableop_18_count_1Identity_18:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_19IdentityRestoreV2:tensors:19"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_19AssignVariableOpassignvariableop_19_totalIdentity_19:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_20IdentityRestoreV2:tensors:20"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_20AssignVariableOpassignvariableop_20_countIdentity_20:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_21IdentityRestoreV2:tensors:21"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_21AssignVariableOp,assignvariableop_21_adamax_dense_18_kernel_mIdentity_21:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_22IdentityRestoreV2:tensors:22"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_22AssignVariableOp*assignvariableop_22_adamax_dense_18_bias_mIdentity_22:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_23IdentityRestoreV2:tensors:23"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_23AssignVariableOp,assignvariableop_23_adamax_dense_19_kernel_mIdentity_23:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_24IdentityRestoreV2:tensors:24"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_24AssignVariableOp*assignvariableop_24_adamax_dense_19_bias_mIdentity_24:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_25IdentityRestoreV2:tensors:25"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_25AssignVariableOp,assignvariableop_25_adamax_dense_20_kernel_mIdentity_25:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_26IdentityRestoreV2:tensors:26"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_26AssignVariableOp*assignvariableop_26_adamax_dense_20_bias_mIdentity_26:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_27IdentityRestoreV2:tensors:27"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_27AssignVariableOp,assignvariableop_27_adamax_dense_21_kernel_mIdentity_27:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_28IdentityRestoreV2:tensors:28"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_28AssignVariableOp*assignvariableop_28_adamax_dense_21_bias_mIdentity_28:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_29IdentityRestoreV2:tensors:29"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_29AssignVariableOp,assignvariableop_29_adamax_dense_22_kernel_mIdentity_29:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_30IdentityRestoreV2:tensors:30"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_30AssignVariableOp*assignvariableop_30_adamax_dense_22_bias_mIdentity_30:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_31IdentityRestoreV2:tensors:31"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_31AssignVariableOp,assignvariableop_31_adamax_dense_23_kernel_mIdentity_31:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_32IdentityRestoreV2:tensors:32"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_32AssignVariableOp*assignvariableop_32_adamax_dense_23_bias_mIdentity_32:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_33IdentityRestoreV2:tensors:33"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_33AssignVariableOp,assignvariableop_33_adamax_dense_18_kernel_vIdentity_33:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_34IdentityRestoreV2:tensors:34"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_34AssignVariableOp*assignvariableop_34_adamax_dense_18_bias_vIdentity_34:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_35IdentityRestoreV2:tensors:35"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_35AssignVariableOp,assignvariableop_35_adamax_dense_19_kernel_vIdentity_35:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_36IdentityRestoreV2:tensors:36"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_36AssignVariableOp*assignvariableop_36_adamax_dense_19_bias_vIdentity_36:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_37IdentityRestoreV2:tensors:37"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_37AssignVariableOp,assignvariableop_37_adamax_dense_20_kernel_vIdentity_37:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_38IdentityRestoreV2:tensors:38"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_38AssignVariableOp*assignvariableop_38_adamax_dense_20_bias_vIdentity_38:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_39IdentityRestoreV2:tensors:39"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_39AssignVariableOp,assignvariableop_39_adamax_dense_21_kernel_vIdentity_39:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_40IdentityRestoreV2:tensors:40"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_40AssignVariableOp*assignvariableop_40_adamax_dense_21_bias_vIdentity_40:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_41IdentityRestoreV2:tensors:41"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_41AssignVariableOp,assignvariableop_41_adamax_dense_22_kernel_vIdentity_41:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_42IdentityRestoreV2:tensors:42"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_42AssignVariableOp*assignvariableop_42_adamax_dense_22_bias_vIdentity_42:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_43IdentityRestoreV2:tensors:43"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_43AssignVariableOp,assignvariableop_43_adamax_dense_23_kernel_vIdentity_43:output:0"/device:CPU:0*
_output_shapes
 *
dtype0_
Identity_44IdentityRestoreV2:tensors:44"/device:CPU:0*
T0*
_output_shapes
:
AssignVariableOp_44AssignVariableOp*assignvariableop_44_adamax_dense_23_bias_vIdentity_44:output:0"/device:CPU:0*
_output_shapes
 *
dtype01
NoOpNoOp"/device:CPU:0*
_output_shapes
 ­
Identity_45Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_33^AssignVariableOp_34^AssignVariableOp_35^AssignVariableOp_36^AssignVariableOp_37^AssignVariableOp_38^AssignVariableOp_39^AssignVariableOp_4^AssignVariableOp_40^AssignVariableOp_41^AssignVariableOp_42^AssignVariableOp_43^AssignVariableOp_44^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: W
Identity_46IdentityIdentity_45:output:0^NoOp_1*
T0*
_output_shapes
: 
NoOp_1NoOp^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_33^AssignVariableOp_34^AssignVariableOp_35^AssignVariableOp_36^AssignVariableOp_37^AssignVariableOp_38^AssignVariableOp_39^AssignVariableOp_4^AssignVariableOp_40^AssignVariableOp_41^AssignVariableOp_42^AssignVariableOp_43^AssignVariableOp_44^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*"
_acd_function_control_output(*
_output_shapes
 "#
identity_46Identity_46:output:0*o
_input_shapes^
\: : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : : 2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132*
AssignVariableOp_14AssignVariableOp_142*
AssignVariableOp_15AssignVariableOp_152*
AssignVariableOp_16AssignVariableOp_162*
AssignVariableOp_17AssignVariableOp_172*
AssignVariableOp_18AssignVariableOp_182*
AssignVariableOp_19AssignVariableOp_192(
AssignVariableOp_2AssignVariableOp_22*
AssignVariableOp_20AssignVariableOp_202*
AssignVariableOp_21AssignVariableOp_212*
AssignVariableOp_22AssignVariableOp_222*
AssignVariableOp_23AssignVariableOp_232*
AssignVariableOp_24AssignVariableOp_242*
AssignVariableOp_25AssignVariableOp_252*
AssignVariableOp_26AssignVariableOp_262*
AssignVariableOp_27AssignVariableOp_272*
AssignVariableOp_28AssignVariableOp_282*
AssignVariableOp_29AssignVariableOp_292(
AssignVariableOp_3AssignVariableOp_32*
AssignVariableOp_30AssignVariableOp_302*
AssignVariableOp_31AssignVariableOp_312*
AssignVariableOp_32AssignVariableOp_322*
AssignVariableOp_33AssignVariableOp_332*
AssignVariableOp_34AssignVariableOp_342*
AssignVariableOp_35AssignVariableOp_352*
AssignVariableOp_36AssignVariableOp_362*
AssignVariableOp_37AssignVariableOp_372*
AssignVariableOp_38AssignVariableOp_382*
AssignVariableOp_39AssignVariableOp_392(
AssignVariableOp_4AssignVariableOp_42*
AssignVariableOp_40AssignVariableOp_402*
AssignVariableOp_41AssignVariableOp_412*
AssignVariableOp_42AssignVariableOp_422*
AssignVariableOp_43AssignVariableOp_432*
AssignVariableOp_44AssignVariableOp_442(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_9:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix

ª
__inference_loss_fn_9_2749013F
8dense_22_bias_regularizer_square_readvariableop_resource:@
identity¢/dense_22/bias/Regularizer/Square/ReadVariableOp¤
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOp8dense_22_bias_regularizer_square_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: _
IdentityIdentity!dense_22/bias/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: x
NoOpNoOp0^dense_22/bias/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp
¡
G
+__inference_dropout_3_layer_call_fn_2748862

inputs
identity±
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747702`
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ç[
ç
 __inference__traced_save_2749171
file_prefix.
*savev2_dense_18_kernel_read_readvariableop,
(savev2_dense_18_bias_read_readvariableop.
*savev2_dense_19_kernel_read_readvariableop,
(savev2_dense_19_bias_read_readvariableop.
*savev2_dense_20_kernel_read_readvariableop,
(savev2_dense_20_bias_read_readvariableop.
*savev2_dense_21_kernel_read_readvariableop,
(savev2_dense_21_bias_read_readvariableop.
*savev2_dense_22_kernel_read_readvariableop,
(savev2_dense_22_bias_read_readvariableop.
*savev2_dense_23_kernel_read_readvariableop,
(savev2_dense_23_bias_read_readvariableop*
&savev2_adamax_iter_read_readvariableop	,
(savev2_adamax_beta_1_read_readvariableop,
(savev2_adamax_beta_2_read_readvariableop+
'savev2_adamax_decay_read_readvariableop3
/savev2_adamax_learning_rate_read_readvariableop&
"savev2_total_1_read_readvariableop&
"savev2_count_1_read_readvariableop$
 savev2_total_read_readvariableop$
 savev2_count_read_readvariableop7
3savev2_adamax_dense_18_kernel_m_read_readvariableop5
1savev2_adamax_dense_18_bias_m_read_readvariableop7
3savev2_adamax_dense_19_kernel_m_read_readvariableop5
1savev2_adamax_dense_19_bias_m_read_readvariableop7
3savev2_adamax_dense_20_kernel_m_read_readvariableop5
1savev2_adamax_dense_20_bias_m_read_readvariableop7
3savev2_adamax_dense_21_kernel_m_read_readvariableop5
1savev2_adamax_dense_21_bias_m_read_readvariableop7
3savev2_adamax_dense_22_kernel_m_read_readvariableop5
1savev2_adamax_dense_22_bias_m_read_readvariableop7
3savev2_adamax_dense_23_kernel_m_read_readvariableop5
1savev2_adamax_dense_23_bias_m_read_readvariableop7
3savev2_adamax_dense_18_kernel_v_read_readvariableop5
1savev2_adamax_dense_18_bias_v_read_readvariableop7
3savev2_adamax_dense_19_kernel_v_read_readvariableop5
1savev2_adamax_dense_19_bias_v_read_readvariableop7
3savev2_adamax_dense_20_kernel_v_read_readvariableop5
1savev2_adamax_dense_20_bias_v_read_readvariableop7
3savev2_adamax_dense_21_kernel_v_read_readvariableop5
1savev2_adamax_dense_21_bias_v_read_readvariableop7
3savev2_adamax_dense_22_kernel_v_read_readvariableop5
1savev2_adamax_dense_22_bias_v_read_readvariableop7
3savev2_adamax_dense_23_kernel_v_read_readvariableop5
1savev2_adamax_dense_23_bias_v_read_readvariableop
savev2_const

identity_1¢MergeV2Checkpointsw
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*Z
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.parta
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: f

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: L

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :f
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: £
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:.*
dtype0*Ì
valueÂB¿.B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-5/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-5/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-5/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPHÉ
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:.*
dtype0*o
valuefBd.B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B £
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0*savev2_dense_18_kernel_read_readvariableop(savev2_dense_18_bias_read_readvariableop*savev2_dense_19_kernel_read_readvariableop(savev2_dense_19_bias_read_readvariableop*savev2_dense_20_kernel_read_readvariableop(savev2_dense_20_bias_read_readvariableop*savev2_dense_21_kernel_read_readvariableop(savev2_dense_21_bias_read_readvariableop*savev2_dense_22_kernel_read_readvariableop(savev2_dense_22_bias_read_readvariableop*savev2_dense_23_kernel_read_readvariableop(savev2_dense_23_bias_read_readvariableop&savev2_adamax_iter_read_readvariableop(savev2_adamax_beta_1_read_readvariableop(savev2_adamax_beta_2_read_readvariableop'savev2_adamax_decay_read_readvariableop/savev2_adamax_learning_rate_read_readvariableop"savev2_total_1_read_readvariableop"savev2_count_1_read_readvariableop savev2_total_read_readvariableop savev2_count_read_readvariableop3savev2_adamax_dense_18_kernel_m_read_readvariableop1savev2_adamax_dense_18_bias_m_read_readvariableop3savev2_adamax_dense_19_kernel_m_read_readvariableop1savev2_adamax_dense_19_bias_m_read_readvariableop3savev2_adamax_dense_20_kernel_m_read_readvariableop1savev2_adamax_dense_20_bias_m_read_readvariableop3savev2_adamax_dense_21_kernel_m_read_readvariableop1savev2_adamax_dense_21_bias_m_read_readvariableop3savev2_adamax_dense_22_kernel_m_read_readvariableop1savev2_adamax_dense_22_bias_m_read_readvariableop3savev2_adamax_dense_23_kernel_m_read_readvariableop1savev2_adamax_dense_23_bias_m_read_readvariableop3savev2_adamax_dense_18_kernel_v_read_readvariableop1savev2_adamax_dense_18_bias_v_read_readvariableop3savev2_adamax_dense_19_kernel_v_read_readvariableop1savev2_adamax_dense_19_bias_v_read_readvariableop3savev2_adamax_dense_20_kernel_v_read_readvariableop1savev2_adamax_dense_20_bias_v_read_readvariableop3savev2_adamax_dense_21_kernel_v_read_readvariableop1savev2_adamax_dense_21_bias_v_read_readvariableop3savev2_adamax_dense_22_kernel_v_read_readvariableop1savev2_adamax_dense_22_bias_v_read_readvariableop3savev2_adamax_dense_23_kernel_v_read_readvariableop1savev2_adamax_dense_23_bias_v_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *<
dtypes2
02.	
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 f
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: Q

Identity_1IdentityIdentity:output:0^NoOp*
T0*
_output_shapes
: [
NoOpNoOp^MergeV2Checkpoints*"
_acd_function_control_output(*
_output_shapes
 "!

identity_1Identity_1:output:0*Ë
_input_shapes¹
¶: :@:@:@@:@:@@:@:@@:@:@@:@:@:: : : : : : : : : :@:@:@@:@:@@:@:@@:@:@@:@:@::@:@:@@:@:@@:@:@@:@:@@:@:@:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:$ 

_output_shapes

:@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$	 

_output_shapes

:@@: 


_output_shapes
:@:$ 

_output_shapes

:@: 

_output_shapes
::

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :$ 

_output_shapes

:@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$ 

_output_shapes

:@@: 

_output_shapes
:@:$  

_output_shapes

:@: !

_output_shapes
::$" 

_output_shapes

:@: #

_output_shapes
:@:$$ 

_output_shapes

:@@: %

_output_shapes
:@:$& 

_output_shapes

:@@: '

_output_shapes
:@:$( 

_output_shapes

:@@: )

_output_shapes
:@:$* 

_output_shapes

:@@: +

_output_shapes
:@:$, 

_output_shapes

:@: -

_output_shapes
::.

_output_shapes
: 

Ü
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
õ
ª
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748584

inputs9
'dense_18_matmul_readvariableop_resource:@6
(dense_18_biasadd_readvariableop_resource:@9
'dense_19_matmul_readvariableop_resource:@@6
(dense_19_biasadd_readvariableop_resource:@9
'dense_20_matmul_readvariableop_resource:@@6
(dense_20_biasadd_readvariableop_resource:@9
'dense_21_matmul_readvariableop_resource:@@6
(dense_21_biasadd_readvariableop_resource:@9
'dense_22_matmul_readvariableop_resource:@@6
(dense_22_biasadd_readvariableop_resource:@9
'dense_23_matmul_readvariableop_resource:@6
(dense_23_biasadd_readvariableop_resource:
identity¢dense_18/BiasAdd/ReadVariableOp¢dense_18/MatMul/ReadVariableOp¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢dense_19/BiasAdd/ReadVariableOp¢dense_19/MatMul/ReadVariableOp¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢dense_20/BiasAdd/ReadVariableOp¢dense_20/MatMul/ReadVariableOp¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢dense_21/BiasAdd/ReadVariableOp¢dense_21/MatMul/ReadVariableOp¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢dense_22/BiasAdd/ReadVariableOp¢dense_22/MatMul/ReadVariableOp¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢dense_23/BiasAdd/ReadVariableOp¢dense_23/MatMul/ReadVariableOp
dense_18/MatMul/ReadVariableOpReadVariableOp'dense_18_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0{
dense_18/MatMulMatMulinputs&dense_18/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_18/BiasAdd/ReadVariableOpReadVariableOp(dense_18_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_18/BiasAddBiasAdddense_18/MatMul:product:0'dense_18/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_18/TanhTanhdense_18/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_19/MatMul/ReadVariableOpReadVariableOp'dense_19_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_19/MatMulMatMuldense_18/Tanh:y:0&dense_19/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_19/BiasAdd/ReadVariableOpReadVariableOp(dense_19_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_19/BiasAddBiasAdddense_19/MatMul:product:0'dense_19/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_19/TanhTanhdense_19/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_20/MatMul/ReadVariableOpReadVariableOp'dense_20_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_20/MatMulMatMuldense_19/Tanh:y:0&dense_20/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_20/BiasAdd/ReadVariableOpReadVariableOp(dense_20_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_20/BiasAddBiasAdddense_20/MatMul:product:0'dense_20/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_20/TanhTanhdense_20/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_21/MatMul/ReadVariableOpReadVariableOp'dense_21_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_21/MatMulMatMuldense_20/Tanh:y:0&dense_21/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_21/BiasAdd/ReadVariableOpReadVariableOp(dense_21_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_21/BiasAddBiasAdddense_21/MatMul:product:0'dense_21/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_21/TanhTanhdense_21/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_22/MatMul/ReadVariableOpReadVariableOp'dense_22_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
dense_22/MatMulMatMuldense_21/Tanh:y:0&dense_22/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_22/BiasAdd/ReadVariableOpReadVariableOp(dense_22_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
dense_22/BiasAddBiasAdddense_22/MatMul:product:0'dense_22/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@b
dense_22/TanhTanhdense_22/BiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@c
dropout_3/IdentityIdentitydense_22/Tanh:y:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
dense_23/MatMul/ReadVariableOpReadVariableOp'dense_23_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0
dense_23/MatMulMatMuldropout_3/Identity:output:0&dense_23/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
dense_23/BiasAdd/ReadVariableOpReadVariableOp(dense_23_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0
dense_23/BiasAddBiasAdddense_23/MatMul:product:0'dense_23/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_18_matmul_readvariableop_resource*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_18_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_19_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_19_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_20_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_20_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_21_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_21_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOp'dense_22_matmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOp(dense_22_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: h
IdentityIdentitydense_23/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿÖ
NoOpNoOp ^dense_18/BiasAdd/ReadVariableOp^dense_18/MatMul/ReadVariableOp0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp ^dense_19/BiasAdd/ReadVariableOp^dense_19/MatMul/ReadVariableOp0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp ^dense_20/BiasAdd/ReadVariableOp^dense_20/MatMul/ReadVariableOp0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp ^dense_21/BiasAdd/ReadVariableOp^dense_21/MatMul/ReadVariableOp0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp ^dense_22/BiasAdd/ReadVariableOp^dense_22/MatMul/ReadVariableOp0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp ^dense_23/BiasAdd/ReadVariableOp^dense_23/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2B
dense_18/BiasAdd/ReadVariableOpdense_18/BiasAdd/ReadVariableOp2@
dense_18/MatMul/ReadVariableOpdense_18/MatMul/ReadVariableOp2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2B
dense_19/BiasAdd/ReadVariableOpdense_19/BiasAdd/ReadVariableOp2@
dense_19/MatMul/ReadVariableOpdense_19/MatMul/ReadVariableOp2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2B
dense_20/BiasAdd/ReadVariableOpdense_20/BiasAdd/ReadVariableOp2@
dense_20/MatMul/ReadVariableOpdense_20/MatMul/ReadVariableOp2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2B
dense_21/BiasAdd/ReadVariableOpdense_21/BiasAdd/ReadVariableOp2@
dense_21/MatMul/ReadVariableOpdense_21/MatMul/ReadVariableOp2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2B
dense_22/BiasAdd/ReadVariableOpdense_22/BiasAdd/ReadVariableOp2@
dense_22/MatMul/ReadVariableOpdense_22/MatMul/ReadVariableOp2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2B
dense_23/BiasAdd/ReadVariableOpdense_23/BiasAdd/ReadVariableOp2@
dense_23/MatMul/ReadVariableOpdense_23/MatMul/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs

Ü
E__inference_dense_21_layer_call_and_return_conditional_losses_2748825

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ä

*__inference_dense_20_layer_call_fn_2748770

inputs
unknown:@@
	unknown_0:@
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
ô	
e
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748884

inputs
identityR
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *ä8?d
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@C
dropout/ShapeShapeinputs*
T0*
_output_shapes
:
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*
dtype0[
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *ÍÌÌ=¦
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@o
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@i
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Y
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
¸
²
__inference_loss_fn_4_2748958L
:dense_20_kernel_regularizer_square_readvariableop_resource:@@
identity¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¬
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOp:dense_20_kernel_regularizer_square_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: a
IdentityIdentity#dense_20/kernel/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: z
NoOpNoOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp
ãq
Ð	
I__inference_sequential_3_layer_call_and_return_conditional_losses_2747781

inputs"
dense_18_2747576:@
dense_18_2747578:@"
dense_19_2747605:@@
dense_19_2747607:@"
dense_20_2747634:@@
dense_20_2747636:@"
dense_21_2747663:@@
dense_21_2747665:@"
dense_22_2747692:@@
dense_22_2747694:@"
dense_23_2747715:@
dense_23_2747717:
identity¢ dense_18/StatefulPartitionedCall¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢ dense_19/StatefulPartitionedCall¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢ dense_20/StatefulPartitionedCall¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢ dense_21/StatefulPartitionedCall¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢ dense_22/StatefulPartitionedCall¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢ dense_23/StatefulPartitionedCalló
 dense_18/StatefulPartitionedCallStatefulPartitionedCallinputsdense_18_2747576dense_18_2747578*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575
 dense_19/StatefulPartitionedCallStatefulPartitionedCall)dense_18/StatefulPartitionedCall:output:0dense_19_2747605dense_19_2747607*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604
 dense_20/StatefulPartitionedCallStatefulPartitionedCall)dense_19/StatefulPartitionedCall:output:0dense_20_2747634dense_20_2747636*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633
 dense_21/StatefulPartitionedCallStatefulPartitionedCall)dense_20/StatefulPartitionedCall:output:0dense_21_2747663dense_21_2747665*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662
 dense_22/StatefulPartitionedCallStatefulPartitionedCall)dense_21/StatefulPartitionedCall:output:0dense_22_2747692dense_22_2747694*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691Þ
dropout_3/PartitionedCallPartitionedCall)dense_22/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747702
 dense_23/StatefulPartitionedCallStatefulPartitionedCall"dropout_3/PartitionedCall:output:0dense_23_2747715dense_23_2747717*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2747576*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2747578*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2747605*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2747607*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2747634*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2747636*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2747663*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2747665*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2747692*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2747694*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: x
IdentityIdentity)dense_23/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
NoOpNoOp!^dense_18/StatefulPartitionedCall0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp!^dense_19/StatefulPartitionedCall0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp!^dense_20/StatefulPartitionedCall0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp!^dense_21/StatefulPartitionedCall0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp!^dense_22/StatefulPartitionedCall0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp!^dense_23/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2D
 dense_18/StatefulPartitionedCall dense_18/StatefulPartitionedCall2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2D
 dense_19/StatefulPartitionedCall dense_19/StatefulPartitionedCall2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2D
 dense_20/StatefulPartitionedCall dense_20/StatefulPartitionedCall2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2D
 dense_21/StatefulPartitionedCall dense_21/StatefulPartitionedCall2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2D
 dense_22/StatefulPartitionedCall dense_22/StatefulPartitionedCall2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2D
 dense_23/StatefulPartitionedCall dense_23/StatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs

ª
__inference_loss_fn_7_2748991F
8dense_21_bias_regularizer_square_readvariableop_resource:@
identity¢/dense_21/bias/Regularizer/Square/ReadVariableOp¤
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOp8dense_21_bias_regularizer_square_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: _
IdentityIdentity!dense_21/bias/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: x
NoOpNoOp0^dense_21/bias/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp

Ü
E__inference_dense_22_layer_call_and_return_conditional_losses_2748857

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ä

*__inference_dense_19_layer_call_fn_2748738

inputs
unknown:@@
	unknown_0:@
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
s
ô	
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748017

inputs"
dense_18_2747925:@
dense_18_2747927:@"
dense_19_2747930:@@
dense_19_2747932:@"
dense_20_2747935:@@
dense_20_2747937:@"
dense_21_2747940:@@
dense_21_2747942:@"
dense_22_2747945:@@
dense_22_2747947:@"
dense_23_2747951:@
dense_23_2747953:
identity¢ dense_18/StatefulPartitionedCall¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¢ dense_19/StatefulPartitionedCall¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOp¢ dense_20/StatefulPartitionedCall¢/dense_20/bias/Regularizer/Square/ReadVariableOp¢1dense_20/kernel/Regularizer/Square/ReadVariableOp¢ dense_21/StatefulPartitionedCall¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOp¢ dense_22/StatefulPartitionedCall¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOp¢ dense_23/StatefulPartitionedCall¢!dropout_3/StatefulPartitionedCalló
 dense_18/StatefulPartitionedCallStatefulPartitionedCallinputsdense_18_2747925dense_18_2747927*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575
 dense_19/StatefulPartitionedCallStatefulPartitionedCall)dense_18/StatefulPartitionedCall:output:0dense_19_2747930dense_19_2747932*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_19_layer_call_and_return_conditional_losses_2747604
 dense_20/StatefulPartitionedCallStatefulPartitionedCall)dense_19/StatefulPartitionedCall:output:0dense_20_2747935dense_20_2747937*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_20_layer_call_and_return_conditional_losses_2747633
 dense_21/StatefulPartitionedCallStatefulPartitionedCall)dense_20/StatefulPartitionedCall:output:0dense_21_2747940dense_21_2747942*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662
 dense_22/StatefulPartitionedCallStatefulPartitionedCall)dense_21/StatefulPartitionedCall:output:0dense_22_2747945dense_22_2747947*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691î
!dropout_3/StatefulPartitionedCallStatefulPartitionedCall)dense_22/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_dropout_3_layer_call_and_return_conditional_losses_2747838
 dense_23/StatefulPartitionedCallStatefulPartitionedCall*dropout_3/StatefulPartitionedCall:output:0dense_23_2747951dense_23_2747953*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_23_layer_call_and_return_conditional_losses_2747714
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2747925*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_18_2747927*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2747930*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_19_2747932*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_20/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2747935*
_output_shapes

:@@*
dtype0
"dense_20/kernel/Regularizer/SquareSquare9dense_20/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_20/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_20/kernel/Regularizer/SumSum&dense_20/kernel/Regularizer/Square:y:0*dense_20/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_20/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/kernel/Regularizer/mulMul*dense_20/kernel/Regularizer/mul/x:output:0(dense_20/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_20/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_20_2747937*
_output_shapes
:@*
dtype0
 dense_20/bias/Regularizer/SquareSquare7dense_20/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_20/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_20/bias/Regularizer/SumSum$dense_20/bias/Regularizer/Square:y:0(dense_20/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_20/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_20/bias/Regularizer/mulMul(dense_20/bias/Regularizer/mul/x:output:0&dense_20/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2747940*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_21_2747942*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2747945*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: |
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpdense_22_2747947*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: x
IdentityIdentity)dense_23/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿº
NoOpNoOp!^dense_18/StatefulPartitionedCall0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp!^dense_19/StatefulPartitionedCall0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp!^dense_20/StatefulPartitionedCall0^dense_20/bias/Regularizer/Square/ReadVariableOp2^dense_20/kernel/Regularizer/Square/ReadVariableOp!^dense_21/StatefulPartitionedCall0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp!^dense_22/StatefulPartitionedCall0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp!^dense_23/StatefulPartitionedCall"^dropout_3/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 2D
 dense_18/StatefulPartitionedCall dense_18/StatefulPartitionedCall2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp2D
 dense_19/StatefulPartitionedCall dense_19/StatefulPartitionedCall2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp2D
 dense_20/StatefulPartitionedCall dense_20/StatefulPartitionedCall2b
/dense_20/bias/Regularizer/Square/ReadVariableOp/dense_20/bias/Regularizer/Square/ReadVariableOp2f
1dense_20/kernel/Regularizer/Square/ReadVariableOp1dense_20/kernel/Regularizer/Square/ReadVariableOp2D
 dense_21/StatefulPartitionedCall dense_21/StatefulPartitionedCall2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp2D
 dense_22/StatefulPartitionedCall dense_22/StatefulPartitionedCall2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp2D
 dense_23/StatefulPartitionedCall dense_23/StatefulPartitionedCall2F
!dropout_3/StatefulPartitionedCall!dropout_3/StatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs

Ü
E__inference_dense_19_layer_call_and_return_conditional_losses_2748761

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_19/bias/Regularizer/Square/ReadVariableOp¢1dense_19/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_19/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_19/kernel/Regularizer/SquareSquare9dense_19/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_19/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_19/kernel/Regularizer/SumSum&dense_19/kernel/Regularizer/Square:y:0*dense_19/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_19/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/kernel/Regularizer/mulMul*dense_19/kernel/Regularizer/mul/x:output:0(dense_19/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_19/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_19/bias/Regularizer/SquareSquare7dense_19/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_19/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_19/bias/Regularizer/SumSum$dense_19/bias/Regularizer/Square:y:0(dense_19/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_19/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_19/bias/Regularizer/mulMul(dense_19/bias/Regularizer/mul/x:output:0&dense_19/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_19/bias/Regularizer/Square/ReadVariableOp2^dense_19/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_19/bias/Regularizer/Square/ReadVariableOp/dense_19/bias/Regularizer/Square/ReadVariableOp2f
1dense_19/kernel/Regularizer/Square/ReadVariableOp1dense_19/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Å

¢
%__inference_signature_wrapper_2748360
input_4
unknown:@
	unknown_0:@
	unknown_1:@@
	unknown_2:@
	unknown_3:@@
	unknown_4:@
	unknown_5:@@
	unknown_6:@
	unknown_7:@@
	unknown_8:@
	unknown_9:@

unknown_10:
identity¢StatefulPartitionedCall»
StatefulPartitionedCallStatefulPartitionedCallinput_4unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *+
f&R$
"__inference__wrapped_model_2747545o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4
¸
²
__inference_loss_fn_0_2748914L
:dense_18_kernel_regularizer_square_readvariableop_resource:@
identity¢1dense_18/kernel/Regularizer/Square/ReadVariableOp¬
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOp:dense_18_kernel_regularizer_square_readvariableop_resource*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: a
IdentityIdentity#dense_18/kernel/Regularizer/mul:z:0^NoOp*
T0*
_output_shapes
: z
NoOpNoOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp

Ü
E__inference_dense_22_layer_call_and_return_conditional_losses_2747691

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_22/bias/Regularizer/Square/ReadVariableOp¢1dense_22/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_22/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_22/kernel/Regularizer/SquareSquare9dense_22/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_22/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_22/kernel/Regularizer/SumSum&dense_22/kernel/Regularizer/Square:y:0*dense_22/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_22/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/kernel/Regularizer/mulMul*dense_22/kernel/Regularizer/mul/x:output:0(dense_22/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_22/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_22/bias/Regularizer/SquareSquare7dense_22/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_22/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_22/bias/Regularizer/SumSum$dense_22/bias/Regularizer/Square:y:0(dense_22/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_22/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_22/bias/Regularizer/mulMul(dense_22/bias/Regularizer/mul/x:output:0&dense_22/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_22/bias/Regularizer/Square/ReadVariableOp2^dense_22/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_22/bias/Regularizer/Square/ReadVariableOp/dense_22/bias/Regularizer/Square/ReadVariableOp2f
1dense_22/kernel/Regularizer/Square/ReadVariableOp1dense_22/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
Ä

*__inference_dense_18_layer_call_fn_2748706

inputs
unknown:@
	unknown_0:@
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_18_layer_call_and_return_conditional_losses_2747575o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
Ä

*__inference_dense_21_layer_call_fn_2748802

inputs
unknown:@@
	unknown_0:@
identity¢StatefulPartitionedCallÚ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs
õ

«
.__inference_sequential_3_layer_call_fn_2747808
input_4
unknown:@
	unknown_0:@
	unknown_1:@@
	unknown_2:@
	unknown_3:@@
	unknown_4:@
	unknown_5:@@
	unknown_6:@
	unknown_7:@@
	unknown_8:@
	unknown_9:@

unknown_10:
identity¢StatefulPartitionedCallâ
StatefulPartitionedCallStatefulPartitionedCallinput_4unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*.
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8 *R
fMRK
I__inference_sequential_3_layer_call_and_return_conditional_losses_2747781o
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*>
_input_shapes-
+:ÿÿÿÿÿÿÿÿÿ: : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
!
_user_specified_name	input_4

Ü
E__inference_dense_18_layer_call_and_return_conditional_losses_2748729

inputs0
matmul_readvariableop_resource:@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_18/bias/Regularizer/Square/ReadVariableOp¢1dense_18/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_18/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0
"dense_18/kernel/Regularizer/SquareSquare9dense_18/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@r
!dense_18/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_18/kernel/Regularizer/SumSum&dense_18/kernel/Regularizer/Square:y:0*dense_18/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_18/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/kernel/Regularizer/mulMul*dense_18/kernel/Regularizer/mul/x:output:0(dense_18/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_18/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_18/bias/Regularizer/SquareSquare7dense_18/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_18/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_18/bias/Regularizer/SumSum$dense_18/bias/Regularizer/Square:y:0(dense_18/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_18/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_18/bias/Regularizer/mulMul(dense_18/bias/Regularizer/mul/x:output:0&dense_18/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_18/bias/Regularizer/Square/ReadVariableOp2^dense_18/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_18/bias/Regularizer/Square/ReadVariableOp/dense_18/bias/Regularizer/Square/ReadVariableOp2f
1dense_18/kernel/Regularizer/Square/ReadVariableOp1dense_18/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ
 
_user_specified_nameinputs
È	
ö
E__inference_dense_23_layer_call_and_return_conditional_losses_2748903

inputs0
matmul_readvariableop_resource:@-
biasadd_readvariableop_resource:
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿr
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ_
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿw
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs

Ü
E__inference_dense_21_layer_call_and_return_conditional_losses_2747662

inputs0
matmul_readvariableop_resource:@@-
biasadd_readvariableop_resource:@
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOp¢/dense_21/bias/Regularizer/Square/ReadVariableOp¢1dense_21/kernel/Regularizer/Square/ReadVariableOpt
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0i
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@r
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0v
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@P
TanhTanhBiasAdd:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
1dense_21/kernel/Regularizer/Square/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:@@*
dtype0
"dense_21/kernel/Regularizer/SquareSquare9dense_21/kernel/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes

:@@r
!dense_21/kernel/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB"       
dense_21/kernel/Regularizer/SumSum&dense_21/kernel/Regularizer/Square:y:0*dense_21/kernel/Regularizer/Const:output:0*
T0*
_output_shapes
: f
!dense_21/kernel/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/kernel/Regularizer/mulMul*dense_21/kernel/Regularizer/mul/x:output:0(dense_21/kernel/Regularizer/Sum:output:0*
T0*
_output_shapes
: 
/dense_21/bias/Regularizer/Square/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype0
 dense_21/bias/Regularizer/SquareSquare7dense_21/bias/Regularizer/Square/ReadVariableOp:value:0*
T0*
_output_shapes
:@i
dense_21/bias/Regularizer/ConstConst*
_output_shapes
:*
dtype0*
valueB: 
dense_21/bias/Regularizer/SumSum$dense_21/bias/Regularizer/Square:y:0(dense_21/bias/Regularizer/Const:output:0*
T0*
_output_shapes
: d
dense_21/bias/Regularizer/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *¬Å'7
dense_21/bias/Regularizer/mulMul(dense_21/bias/Regularizer/mul/x:output:0&dense_21/bias/Regularizer/Sum:output:0*
T0*
_output_shapes
: W
IdentityIdentityTanh:y:0^NoOp*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@Ý
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp0^dense_21/bias/Regularizer/Square/ReadVariableOp2^dense_21/kernel/Regularizer/Square/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:ÿÿÿÿÿÿÿÿÿ@: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp2b
/dense_21/bias/Regularizer/Square/ReadVariableOp/dense_21/bias/Regularizer/Square/ReadVariableOp2f
1dense_21/kernel/Regularizer/Square/ReadVariableOp1dense_21/kernel/Regularizer/Square/ReadVariableOp:O K
'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ@
 
_user_specified_nameinputs"¿L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*«
serving_default
;
input_40
serving_default_input_4:0ÿÿÿÿÿÿÿÿÿ<
dense_230
StatefulPartitionedCall:0ÿÿÿÿÿÿÿÿÿtensorflow/serving/predict:îå
Ý
layer_with_weights-0
layer-0
layer_with_weights-1
layer-1
layer_with_weights-2
layer-2
layer_with_weights-3
layer-3
layer_with_weights-4
layer-4
layer-5
layer_with_weights-5
layer-6
	variables
	trainable_variables

regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses
_default_save_signature
	optimizer

signatures"
_tf_keras_sequential
»
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
»
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses

kernel
 bias"
_tf_keras_layer
»
!	variables
"trainable_variables
#regularization_losses
$	keras_api
%__call__
*&&call_and_return_all_conditional_losses

'kernel
(bias"
_tf_keras_layer
»
)	variables
*trainable_variables
+regularization_losses
,	keras_api
-__call__
*.&call_and_return_all_conditional_losses

/kernel
0bias"
_tf_keras_layer
»
1	variables
2trainable_variables
3regularization_losses
4	keras_api
5__call__
*6&call_and_return_all_conditional_losses

7kernel
8bias"
_tf_keras_layer
¼
9	variables
:trainable_variables
;regularization_losses
<	keras_api
=__call__
*>&call_and_return_all_conditional_losses
?_random_generator"
_tf_keras_layer
»
@	variables
Atrainable_variables
Bregularization_losses
C	keras_api
D__call__
*E&call_and_return_all_conditional_losses

Fkernel
Gbias"
_tf_keras_layer
v
0
1
2
 3
'4
(5
/6
07
78
89
F10
G11"
trackable_list_wrapper
v
0
1
2
 3
'4
(5
/6
07
78
89
F10
G11"
trackable_list_wrapper
f
H0
I1
J2
K3
L4
M5
N6
O7
P8
Q9"
trackable_list_wrapper
Ê
Rnon_trainable_variables

Slayers
Tmetrics
Ulayer_regularization_losses
Vlayer_metrics
	variables
	trainable_variables

regularization_losses
__call__
_default_save_signature
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses"
_generic_user_object
î
Wtrace_0
Xtrace_1
Ytrace_2
Ztrace_32
.__inference_sequential_3_layer_call_fn_2747808
.__inference_sequential_3_layer_call_fn_2748449
.__inference_sequential_3_layer_call_fn_2748478
.__inference_sequential_3_layer_call_fn_2748073À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 zWtrace_0zXtrace_1zYtrace_2zZtrace_3
Ú
[trace_0
\trace_1
]trace_2
^trace_32ï
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748584
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748697
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748168
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748263À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 z[trace_0z\trace_1z]trace_2z^trace_3
ÍBÊ
"__inference__wrapped_model_2747545input_4"
²
FullArgSpec
args 
varargsjargs
varkwjkwargs
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
Ã
_iter

`beta_1

abeta_2
	bdecay
clearning_ratem­m®m¯ m°'m±(m²/m³0m´7mµ8m¶Fm·Gm¸v¹vºv» v¼'v½(v¾/v¿0vÀ7vÁ8vÂFvÃGvÄ"
	optimizer
,
dserving_default"
signature_map
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
.
H0
I1"
trackable_list_wrapper
­
enon_trainable_variables

flayers
gmetrics
hlayer_regularization_losses
ilayer_metrics
	variables
trainable_variables
regularization_losses
__call__
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses"
_generic_user_object
î
jtrace_02Ñ
*__inference_dense_18_layer_call_fn_2748706¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zjtrace_0

ktrace_02ì
E__inference_dense_18_layer_call_and_return_conditional_losses_2748729¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zktrace_0
!:@2dense_18/kernel
:@2dense_18/bias
.
0
 1"
trackable_list_wrapper
.
0
 1"
trackable_list_wrapper
.
J0
K1"
trackable_list_wrapper
­
lnon_trainable_variables

mlayers
nmetrics
olayer_regularization_losses
player_metrics
	variables
trainable_variables
regularization_losses
__call__
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses"
_generic_user_object
î
qtrace_02Ñ
*__inference_dense_19_layer_call_fn_2748738¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zqtrace_0

rtrace_02ì
E__inference_dense_19_layer_call_and_return_conditional_losses_2748761¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zrtrace_0
!:@@2dense_19/kernel
:@2dense_19/bias
.
'0
(1"
trackable_list_wrapper
.
'0
(1"
trackable_list_wrapper
.
L0
M1"
trackable_list_wrapper
­
snon_trainable_variables

tlayers
umetrics
vlayer_regularization_losses
wlayer_metrics
!	variables
"trainable_variables
#regularization_losses
%__call__
*&&call_and_return_all_conditional_losses
&&"call_and_return_conditional_losses"
_generic_user_object
î
xtrace_02Ñ
*__inference_dense_20_layer_call_fn_2748770¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zxtrace_0

ytrace_02ì
E__inference_dense_20_layer_call_and_return_conditional_losses_2748793¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 zytrace_0
!:@@2dense_20/kernel
:@2dense_20/bias
.
/0
01"
trackable_list_wrapper
.
/0
01"
trackable_list_wrapper
.
N0
O1"
trackable_list_wrapper
­
znon_trainable_variables

{layers
|metrics
}layer_regularization_losses
~layer_metrics
)	variables
*trainable_variables
+regularization_losses
-__call__
*.&call_and_return_all_conditional_losses
&."call_and_return_conditional_losses"
_generic_user_object
î
trace_02Ñ
*__inference_dense_21_layer_call_fn_2748802¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0

trace_02ì
E__inference_dense_21_layer_call_and_return_conditional_losses_2748825¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0
!:@@2dense_21/kernel
:@2dense_21/bias
.
70
81"
trackable_list_wrapper
.
70
81"
trackable_list_wrapper
.
P0
Q1"
trackable_list_wrapper
²
non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
1	variables
2trainable_variables
3regularization_losses
5__call__
*6&call_and_return_all_conditional_losses
&6"call_and_return_conditional_losses"
_generic_user_object
ð
trace_02Ñ
*__inference_dense_22_layer_call_fn_2748834¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0

trace_02ì
E__inference_dense_22_layer_call_and_return_conditional_losses_2748857¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0
!:@@2dense_22/kernel
:@2dense_22/bias
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
²
non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
9	variables
:trainable_variables
;regularization_losses
=__call__
*>&call_and_return_all_conditional_losses
&>"call_and_return_conditional_losses"
_generic_user_object
Ì
trace_0
trace_12
+__inference_dropout_3_layer_call_fn_2748862
+__inference_dropout_3_layer_call_fn_2748867´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 ztrace_0ztrace_1

trace_0
trace_12Ç
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748872
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748884´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 ztrace_0ztrace_1
"
_generic_user_object
.
F0
G1"
trackable_list_wrapper
.
F0
G1"
trackable_list_wrapper
 "
trackable_list_wrapper
²
non_trainable_variables
layers
metrics
 layer_regularization_losses
layer_metrics
@	variables
Atrainable_variables
Bregularization_losses
D__call__
*E&call_and_return_all_conditional_losses
&E"call_and_return_conditional_losses"
_generic_user_object
ð
trace_02Ñ
*__inference_dense_23_layer_call_fn_2748893¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0

trace_02ì
E__inference_dense_23_layer_call_and_return_conditional_losses_2748903¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 ztrace_0
!:@2dense_23/kernel
:2dense_23/bias
Ð
trace_02±
__inference_loss_fn_0_2748914
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_1_2748925
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_2_2748936
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_3_2748947
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_4_2748958
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_5_2748969
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_6_2748980
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
trace_02±
__inference_loss_fn_7_2748991
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ ztrace_0
Ð
 trace_02±
__inference_loss_fn_8_2749002
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ z trace_0
Ð
¡trace_02±
__inference_loss_fn_9_2749013
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ z¡trace_0
 "
trackable_list_wrapper
Q
0
1
2
3
4
5
6"
trackable_list_wrapper
0
¢0
£1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
Bþ
.__inference_sequential_3_layer_call_fn_2747808input_4"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
Bý
.__inference_sequential_3_layer_call_fn_2748449inputs"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
Bý
.__inference_sequential_3_layer_call_fn_2748478inputs"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
Bþ
.__inference_sequential_3_layer_call_fn_2748073input_4"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748584inputs"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748697inputs"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748168input_4"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748263input_4"À
·²³
FullArgSpec1
args)&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
p 

 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
:	 (2Adamax/iter
: (2Adamax/beta_1
: (2Adamax/beta_2
: (2Adamax/decay
: (2Adamax/learning_rate
ÌBÉ
%__inference_signature_wrapper_2748360input_4"
²
FullArgSpec
args 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
.
H0
I1"
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_18_layer_call_fn_2748706inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_18_layer_call_and_return_conditional_losses_2748729inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
.
J0
K1"
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_19_layer_call_fn_2748738inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_19_layer_call_and_return_conditional_losses_2748761inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
.
L0
M1"
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_20_layer_call_fn_2748770inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_20_layer_call_and_return_conditional_losses_2748793inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
.
N0
O1"
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_21_layer_call_fn_2748802inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_21_layer_call_and_return_conditional_losses_2748825inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
.
P0
Q1"
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_22_layer_call_fn_2748834inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_22_layer_call_and_return_conditional_losses_2748857inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ñBî
+__inference_dropout_3_layer_call_fn_2748862inputs"´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
ñBî
+__inference_dropout_3_layer_call_fn_2748867inputs"´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748872inputs"´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
B
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748884inputs"´
«²§
FullArgSpec)
args!
jself
jinputs

jtraining
varargs
 
varkw
 
defaults
p 

kwonlyargs 
kwonlydefaultsª 
annotationsª *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ÞBÛ
*__inference_dense_23_layer_call_fn_2748893inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
ùBö
E__inference_dense_23_layer_call_and_return_conditional_losses_2748903inputs"¢
²
FullArgSpec
args
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
´B±
__inference_loss_fn_0_2748914"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_1_2748925"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_2_2748936"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_3_2748947"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_4_2748958"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_5_2748969"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_6_2748980"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_7_2748991"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_8_2749002"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
´B±
__inference_loss_fn_9_2749013"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *¢ 
R
¤	variables
¥	keras_api

¦total

§count"
_tf_keras_metric
c
¨	variables
©	keras_api

ªtotal

«count
¬
_fn_kwargs"
_tf_keras_metric
0
¦0
§1"
trackable_list_wrapper
.
¤	variables"
_generic_user_object
:  (2total
:  (2count
0
ª0
«1"
trackable_list_wrapper
.
¨	variables"
_generic_user_object
:  (2total
:  (2count
 "
trackable_dict_wrapper
(:&@2Adamax/dense_18/kernel/m
": @2Adamax/dense_18/bias/m
(:&@@2Adamax/dense_19/kernel/m
": @2Adamax/dense_19/bias/m
(:&@@2Adamax/dense_20/kernel/m
": @2Adamax/dense_20/bias/m
(:&@@2Adamax/dense_21/kernel/m
": @2Adamax/dense_21/bias/m
(:&@@2Adamax/dense_22/kernel/m
": @2Adamax/dense_22/bias/m
(:&@2Adamax/dense_23/kernel/m
": 2Adamax/dense_23/bias/m
(:&@2Adamax/dense_18/kernel/v
": @2Adamax/dense_18/bias/v
(:&@@2Adamax/dense_19/kernel/v
": @2Adamax/dense_19/bias/v
(:&@@2Adamax/dense_20/kernel/v
": @2Adamax/dense_20/bias/v
(:&@@2Adamax/dense_21/kernel/v
": @2Adamax/dense_21/bias/v
(:&@@2Adamax/dense_22/kernel/v
": @2Adamax/dense_22/bias/v
(:&@2Adamax/dense_23/kernel/v
": 2Adamax/dense_23/bias/v
"__inference__wrapped_model_2747545u '(/078FG0¢-
&¢#
!
input_4ÿÿÿÿÿÿÿÿÿ
ª "3ª0
.
dense_23"
dense_23ÿÿÿÿÿÿÿÿÿ¥
E__inference_dense_18_layer_call_and_return_conditional_losses_2748729\/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 }
*__inference_dense_18_layer_call_fn_2748706O/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ
ª "ÿÿÿÿÿÿÿÿÿ@¥
E__inference_dense_19_layer_call_and_return_conditional_losses_2748761\ /¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 }
*__inference_dense_19_layer_call_fn_2748738O /¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "ÿÿÿÿÿÿÿÿÿ@¥
E__inference_dense_20_layer_call_and_return_conditional_losses_2748793\'(/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 }
*__inference_dense_20_layer_call_fn_2748770O'(/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "ÿÿÿÿÿÿÿÿÿ@¥
E__inference_dense_21_layer_call_and_return_conditional_losses_2748825\/0/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 }
*__inference_dense_21_layer_call_fn_2748802O/0/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "ÿÿÿÿÿÿÿÿÿ@¥
E__inference_dense_22_layer_call_and_return_conditional_losses_2748857\78/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 }
*__inference_dense_22_layer_call_fn_2748834O78/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "ÿÿÿÿÿÿÿÿÿ@¥
E__inference_dense_23_layer_call_and_return_conditional_losses_2748903\FG/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ
 }
*__inference_dense_23_layer_call_fn_2748893OFG/¢,
%¢"
 
inputsÿÿÿÿÿÿÿÿÿ@
ª "ÿÿÿÿÿÿÿÿÿ¦
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748872\3¢0
)¢&
 
inputsÿÿÿÿÿÿÿÿÿ@
p 
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 ¦
F__inference_dropout_3_layer_call_and_return_conditional_losses_2748884\3¢0
)¢&
 
inputsÿÿÿÿÿÿÿÿÿ@
p
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ@
 ~
+__inference_dropout_3_layer_call_fn_2748862O3¢0
)¢&
 
inputsÿÿÿÿÿÿÿÿÿ@
p 
ª "ÿÿÿÿÿÿÿÿÿ@~
+__inference_dropout_3_layer_call_fn_2748867O3¢0
)¢&
 
inputsÿÿÿÿÿÿÿÿÿ@
p
ª "ÿÿÿÿÿÿÿÿÿ@<
__inference_loss_fn_0_2748914¢

¢ 
ª " <
__inference_loss_fn_1_2748925¢

¢ 
ª " <
__inference_loss_fn_2_2748936¢

¢ 
ª " <
__inference_loss_fn_3_2748947 ¢

¢ 
ª " <
__inference_loss_fn_4_2748958'¢

¢ 
ª " <
__inference_loss_fn_5_2748969(¢

¢ 
ª " <
__inference_loss_fn_6_2748980/¢

¢ 
ª " <
__inference_loss_fn_7_27489910¢

¢ 
ª " <
__inference_loss_fn_8_27490027¢

¢ 
ª " <
__inference_loss_fn_9_27490138¢

¢ 
ª " ¼
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748168o '(/078FG8¢5
.¢+
!
input_4ÿÿÿÿÿÿÿÿÿ
p 

 
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ
 ¼
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748263o '(/078FG8¢5
.¢+
!
input_4ÿÿÿÿÿÿÿÿÿ
p

 
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ
 »
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748584n '(/078FG7¢4
-¢*
 
inputsÿÿÿÿÿÿÿÿÿ
p 

 
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ
 »
I__inference_sequential_3_layer_call_and_return_conditional_losses_2748697n '(/078FG7¢4
-¢*
 
inputsÿÿÿÿÿÿÿÿÿ
p

 
ª "%¢"

0ÿÿÿÿÿÿÿÿÿ
 
.__inference_sequential_3_layer_call_fn_2747808b '(/078FG8¢5
.¢+
!
input_4ÿÿÿÿÿÿÿÿÿ
p 

 
ª "ÿÿÿÿÿÿÿÿÿ
.__inference_sequential_3_layer_call_fn_2748073b '(/078FG8¢5
.¢+
!
input_4ÿÿÿÿÿÿÿÿÿ
p

 
ª "ÿÿÿÿÿÿÿÿÿ
.__inference_sequential_3_layer_call_fn_2748449a '(/078FG7¢4
-¢*
 
inputsÿÿÿÿÿÿÿÿÿ
p 

 
ª "ÿÿÿÿÿÿÿÿÿ
.__inference_sequential_3_layer_call_fn_2748478a '(/078FG7¢4
-¢*
 
inputsÿÿÿÿÿÿÿÿÿ
p

 
ª "ÿÿÿÿÿÿÿÿÿª
%__inference_signature_wrapper_2748360 '(/078FG;¢8
¢ 
1ª.
,
input_4!
input_4ÿÿÿÿÿÿÿÿÿ"3ª0
.
dense_23"
dense_23ÿÿÿÿÿÿÿÿÿ