FasdUAS 1.101.10   ��   ��    k             l     ��  ��      Log aktualisieren     � 	 	 $   L o g   a k t u a l i s i e r e n   
  
 l     ����  r         b         l     ����  c         o     ���� 0 homeordnerpfad    m    ��
�� 
TEXT��  ��    m       �     T W I _ M a s t e r _ l o g . c  o      ���� 0 filepfad  ��  ��        l     ��  ��    > 8set filepfad to (homeordnerpfad as string) & "version.c"     �   p s e t   f i l e p f a d   t o   ( h o m e o r d n e r p f a d   a s   s t r i n g )   &   " v e r s i o n . c "      l     ��������  ��  ��        l    ����  I   ��  ��
�� .sysodlogaskr        TEXT   b     ! " ! m    	 # # � $ $  l o g   f i l e p f a d :   " o   	 
���� 0 filepfad  ��  ��  ��     % & % l    '���� ' I   ������
�� .miscactvnull��� ��� null��  ��  ��  ��   &  ( ) ( l   " *���� * r    " + , + l     -���� - I    �� . /
�� .rdwropenshor       file . 4    �� 0
�� 
file 0 o    ���� 0 filepfad   / �� 1��
�� 
perm 1 m    ��
�� boovtrue��  ��  ��   , o      ���� 0 	logrefnum 	LogRefNum��  ��   )  2 3 2 l     �� 4 5��   4 . (display dialog "LogRefNum: " & LogRefNum    5 � 6 6 P d i s p l a y   d i a l o g   " L o g R e f N u m :   "   &   L o g R e f N u m 3  7 8 7 l  # � 9���� 9 Q   # � : ; < : k   & � = =  > ? > I  & +�� @��
�� .sysodlogaskr        TEXT @ m   & ' A A � B B  S t a r t   l o g��   ?  C D C r   , 3 E F E l  , 1 G���� G I  , 1�� H��
�� .rdwrread****        **** H o   , -���� 0 	logrefnum 	LogRefNum��  ��  ��   F o      ���� "0 logfilecontents logfileContents D  I J I l  4 4�� K L��   K : 4display dialog "inhalt: " & return & logfileContents    L � M M h d i s p l a y   d i a l o g   " i n h a l t :   "   &   r e t u r n   &   l o g f i l e C o n t e n t s J  N O N I  4 ;�� P��
�� .sysodlogaskr        TEXT P b   4 7 Q R Q m   4 5 S S � T T   n e u e s   L o g D a t u m :   R o   5 6���� 0 neueslogdatum neuesLogDatum��   O  U V U r   < F W X W n   < B Y Z Y 4   = B�� [
�� 
cpar [ m   @ A����  Z o   < =���� "0 logfilecontents logfileContents X o      ���� 0 titel Titel V  \ ] \ I  G R�� ^��
�� .sysodlogaskr        TEXT ^ b   G N _ ` _ m   G J a a � b b  T t e l :   ` o   J M���� 0 titel Titel��   ]  c d c r   S b e f e n   S ^ g h g m   Z ^��
�� 
nmbr h n   S Z i j i 2  V Z��
�� 
cha  j o   S V���� 0 titel Titel f o      ���� 0 l   d  k l k l  c p m n o m r   c p p q p b   c l r s r b   c j t u t o   c f���� 0 projektname Projektname u m   f i v v � w w  L o g   v o m :   s o   j k���� 0 neueslogdatum neuesLogDatum q o      ���� 0 	neuertext 	neuerText n F @& return & neueVersion & "\"" & Version1 & "." & Version2 & "\""    o � x x � &   r e t u r n   &   n e u e V e r s i o n   &   " \ " "   &   V e r s i o n 1   &   " . "   &   V e r s i o n 2   &   " \ " " l  y z y l  q q�� { |��   { 4 .set paragraph 2 of fileContents to neueVersion    | � } } \ s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e V e r s i o n z  ~  ~ I  q ��� ���
�� .sysodlogaskr        TEXT � b   q | � � � b   q x � � � m   q t � � � � �  n e u e r     T i t e l :   � o   t w��
�� 
ret  � o   x {���� 0 	neuertext 	neuerText��     � � � I  � ��� � �
�� .rdwrseofnull���     **** � o   � ����� 0 refnum RefNum � �� ���
�� 
set2 � m   � �����  ��   �  � � � l  � ��� � ���   �  write neuerText to RefNum    � � � � 2 w r i t e   n e u e r T e x t   t o   R e f N u m �  ��� � I  � ��� ���
�� .rdwrclosnull���     **** � o   � ����� 0 	logrefnum 	LogRefNum��  ��   ; R      ������
�� .ascrerr ****      � ****��  ��   < k   � � � �  � � � l  � ���������  ��  ��   �  ��� � I  � ��� ���
�� .rdwrclosnull���     **** � o   � ����� 0 	logrefnum 	LogRefNum��  ��  ��  ��   8  ��� � l     ��������  ��  ��  ��       �� � ���   � ��
�� .aevtoappnull  �   � **** � �� ����� � ���
�� .aevtoappnull  �   � **** � k     � � �  
 � �   � �  % � �  ( � �  7����  ��  ��   �   � !���� �� #������������ A���� S������ a�������� v�� ����������������� 0 homeordnerpfad  
�� 
TEXT�� 0 filepfad  
�� .sysodlogaskr        TEXT
�� .miscactvnull��� ��� null
�� 
file
�� 
perm
�� .rdwropenshor       file�� 0 	logrefnum 	LogRefNum
�� .rdwrread****        ****�� "0 logfilecontents logfileContents�� 0 neueslogdatum neuesLogDatum
�� 
cpar�� 0 titel Titel
�� 
cha 
�� 
nmbr�� 0 l  �� 0 projektname Projektname�� 0 	neuertext 	neuerText
�� 
ret �� 0 refnum RefNum
�� 
set2
�� .rdwrseofnull���     ****
�� .rdwrclosnull���     ****��  ��  �� ���&�%E�O��%j O*j O*��/�el 	E�O q�j O�j E�O��%j O�a k/E` Oa _ %j O_ a -a ,E` O_ a %�%E` Oa _ %_ %j O_ a jl O�j W X   �j ascr  ��ޭ