FasdUAS 1.101.10   ��   ��    k             l   ] ����  O    ]  	  k   \ 
 
     l   ��  ��     activate     �    a c t i v a t e      l   ��������  ��  ��        r        m       �      o      ���� 0 filecontents fileContents      r        4    �� 
�� 
alis  l  
  ����  l  
  ����  I  
 ��   
�� .earsffdralis        afdr   f   
    �� !��
�� 
rtyp ! m    ��
�� 
ctxt��  ��  ��  ��  ��    o      ���� 0 
homeordner     " # " l   �� $ %��   $ . (set homeordner to (path to me as string)    % � & & P s e t   h o m e o r d n e r   t o   ( p a t h   t o   m e   a s   s t r i n g ) #  ' ( ' I   �� )��
�� .sysodlogaskr        TEXT ) b     * + * m     , , � - -  h o m e o r d n e r :   + o    ���� 0 
homeordner  ��   (  . / . l   �� 0 1��   0 3 -set homeordnerpfad to container of homeordner    1 � 2 2 Z s e t   h o m e o r d n e r p f a d   t o   c o n t a i n e r   o f   h o m e o r d n e r /  3 4 3 r    ' 5 6 5 c    % 7 8 7 l   # 9���� 9 n    # : ; : m   ! #��
�� 
ctnr ; 4    !�� <
�� 
cobj < o     ���� 0 
homeordner  ��  ��   8 m   # $��
�� 
ctxt 6 o      ���� 0 homeordnerpfad   4  = > = l  ( (�� ? @��   ? 2 ,set main to file "datum.c" of homeordnerpfad    @ � A A X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d >  B C B r   ( / D E D b   ( - F G F l  ( + H���� H c   ( + I J I o   ( )���� 0 homeordnerpfad   J m   ) *��
�� 
TEXT��  ��   G m   + , K K � L L  d a t u m . c E o      ���� 0 filepfad   C  M N M I  0 9�� O��
�� .sysodlogaskr        TEXT O b   0 5 P Q P m   0 3 R R � S S  f i l e p f a d :   Q o   3 4���� 0 filepfad  ��   N  T U T l  : :�� V W��   V ! tell application "TextEdit"    W � X X 6 t e l l   a p p l i c a t i o n   " T e x t E d i t " U  Y Z Y I  : ?������
�� .miscactvnull��� ��� obj ��  ��   Z  [ \ [ r   @ I ] ^ ] I  @ E������
�� .misccurdldt    ��� null��  ��   ^ o      ���� 	0 heute   \  _ ` _ l  J J�� a b��   a &  display dialog "heute: " & heute    b � c c @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e `  d e d r   J U f g f n   J Q h i h 1   M Q��
�� 
year i o   J M���� 	0 heute   g o      ���� 0 jahrtext   e  j k j r   V a l m l n   V ] n o n m   Y ]��
�� 
mnth o o   V Y���� 	0 heute   m o      ���� 0 	monattext   k  p q p l  b b�� r s��   r * $display dialog "monat: " & monattext    s � t t H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x t q  u v u r   b } w x w n   b y y z y 7  m y�� { |
�� 
ctxt { m   q u������ | m   v x������ z l  b m }���� } b   b m ~  ~ m   b e � � � � �  0  n   e l � � � 1   h l��
�� 
day  � o   e h���� 	0 heute  ��  ��   x o      ���� 0 tag   v  � � � l  ~ ~�� � ���   � " display dialog "tag: " & tag    � � � � 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g �  � � � r   ~ � � � � J   ~ � � �  � � � m   ~ ���
�� 
jan  �  � � � m   � ���
�� 
feb  �  � � � m   � ���
�� 
mar  �  � � � l 	 � � ����� � m   � ���
�� 
apr ��  ��   �  � � � m   � ���
�� 
may  �  � � � m   � ���
�� 
jun  �  � � � m   � ���
�� 
jul  �  � � � m   � ���
�� 
aug  �  � � � l 	 � � ����� � m   � ���
�� 
sep ��  ��   �  � � � m   � ���
�� 
oct  �  � � � m   � ���
�� 
nov  �  ��� � m   � ���
�� 
dec ��   � o      ���� 0 monatsliste MonatsListe �  � � � Y   � � ��� � ��� � Z   � � � ����� � =   � � � � � o   � ����� 0 	monattext   � n   � � � � � 4   � ��� �
�� 
cobj � o   � ����� 0 i   � o   � ����� 0 monatsliste MonatsListe � k   � � � �  � � � r   � � � � � n   � � � � � 7  � ��� � �
�� 
ctxt � m   � ������� � m   � ������� � l  � � ����� � b   � � � � � m   � � � � � � �  0 � o   � ����� 0 i  ��  ��   � o      ���� 	0 monat   �  ��� � l  � � � � � �  S   � � � - ' wenn true, wird die Schleife verlassen    � � � � N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n��  ��  ��  �� 0 i   � m   � �����  � m   � ����� ��   �  � � � l  � ��� � ���   � &  display dialog "monat: " & monat    � � � � @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t �  � � � r   � � � � � l 	 � � ����� � l  � � ����� � n  � � � � � 7  � ��� � �
�� 
cha  � m   � �����  � m   � �����  � l  � � ����� � c   � � � � � o   � ����� 0 jahrtext   � m   � ���
�� 
ctxt��  ��  ��  ��  ��  ��   � o      ���� 0 jahr   �  � � � l  � ��� � ���   � ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext    � � � � r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t �  � � � l  � ���������  ��  ��   �  � � � r   � � � � l  � ����� � I  ��� � �
�� .rdwropenshor       file � 4   ��� �
�� 
file � o  ���� 0 filepfad   � �� ���
�� 
perm � m  ��
�� boovtrue��  ��  ��   � o      ���� 0 refnum RefNum �  � � � Q   � � � � k    � �  � � � r   � � � l  ����� � I � ��~
� .rdwrread****        **** � o  �}�} 0 refnum RefNum�~  ��  ��   � o      �|�| 0 filecontents fileContents �  � � � Z  � � ��{ � � =  " � � � o  �z�z 0 filecontents fileContents � m  ! � � � � �   � k  %t � �  � � � I %,�y �x
�y .sysodlogaskr        TEXT  m  %( �   d a t u m . c   i s t   l e e r�x   �  r  -4 m  -0 �  # d e f i n e   D A T U M   o      �w�w 0 
neuesdatum 
neuesDatum 	
	 l 55�v�v   2 ,display dialog "neuesDatum A: " & neuesDatum    � X d i s p l a y   d i a l o g   " n e u e s D a t u m   A :   "   &   n e u e s D a t u m
  r  5X b  5T b  5P b  5L b  5H b  5D b  5@ b  5< o  58�u�u 0 
neuesdatum 
neuesDatum m  8;   �!!  " o  <?�t�t 0 tag   m  @C"" �##  . o  DG�s�s 	0 monat   m  HK$$ �%%  . o  LO�r�r 0 jahrtext   m  PS&& �''  " o      �q�q 0 
neuesdatum 
neuesDatum ()( I Yd�p*�o
�p .sysodlogaskr        TEXT* b  Y`+,+ m  Y\-- �..  n e u e s D a t u m :  , o  \_�n�n 0 
neuesdatum 
neuesDatum�o  ) /�m/ r  et010 b  ep232 b  el454 m  eh66 �77  / / d a t u m . c5 o  hk�l
�l 
ret 3 o  lo�k�k 0 
neuesdatum 
neuesDatum1 o      �j�j 0 	neuertext 	neuerText�m  �{   � k  w�88 9:9 l ww�i;<�i  ; 7 1display dialog "inhalt: " & return & fileContents   < �== b d i s p l a y   d i a l o g   " i n h a l t :   "   &   r e t u r n   &   f i l e C o n t e n t s: >?> r  w�@A@ n  w}BCB 4  x}�hD
�h 
cparD m  {|�g�g C o  wx�f�f 0 filecontents fileContentsA o      �e�e 0 datum Datum? EFE l ���dGH�d  G &  display dialog "Datum: " & Datum   H �II @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u mF JKJ l ���c�b�a�c  �b  �a  K LML r  ��NON n  ��PQP m  ���`
�` 
nmbrQ n  ��RSR 2 ���_
�_ 
cha S o  ���^�^ 0 datum DatumO o      �]�] 0 l  M TUT l ���\VW�\  V 1 +set neuesDatum to text -l thru -13 of Datum   W �XX V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u mU YZY l ��[\][ r  ��^_^ n  ��`a` 7 ���[bc
�[ 
ctxtb m  ���Z�Z c m  ���Y�Y a o  ���X�X 0 datum Datum_ o      �W�W 0 
neuesdatum 
neuesDatum\ $  Anfang bis und mit Leerschlag   ] �dd <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a gZ efe l ���Vgh�V  g 2 ,display dialog "neuesDatum A: " & neuesDatum   h �ii X d i s p l a y   d i a l o g   " n e u e s D a t u m   A :   "   &   n e u e s D a t u mf jkj r  ��lml b  ��non b  ��pqp b  ��rsr b  ��tut b  ��vwv b  ��xyx b  ��z{z o  ���U�U 0 
neuesdatum 
neuesDatum{ m  ��|| �}}  "y o  ���T�T 0 tag  w m  ��~~ �  .u o  ���S�S 	0 monat  s m  ���� ���  .q o  ���R�R 0 jahrtext  o m  ���� ���  "m o      �Q�Q 0 
neuesdatum 
neuesDatumk ��� l ���P���P  � 0 *display dialog "neuesDatum: " & neuesDatum   � ��� T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m� ��� r  ����� b  ����� b  ����� n  ����� 4  ���O�
�O 
cpar� m  ���N�N � o  ���M�M 0 filecontents fileContents� o  ���L
�L 
ret � o  ���K�K 0 
neuesdatum 
neuesDatum� o      �J�J 0 	neuertext 	neuerText� ��� l ���I���I  � 3 -set paragraph 2 of fileContents to neuesDatum   � ��� Z s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e s D a t u m� ��H� l ���G���G  � 9 3display dialog "neues Datum: " & return & neuerText   � ��� f d i s p l a y   d i a l o g   " n e u e s   D a t u m :   "   &   r e t u r n   &   n e u e r T e x t�H   � ��� l ���F�E�D�F  �E  �D  � ��� I ���C��
�C .rdwrseofnull���     ****� o  ���B�B 0 refnum RefNum� �A��@
�A 
set2� m  ���?�?  �@  � ��� I ���>��
�> .rdwrwritnull���     ****� o  ���=�= 0 	neuertext 	neuerText� �<��;
�< 
refn� o  ���:�: 0 refnum RefNum�;  � ��9� I � �8��7
�8 .rdwrclosnull���     ****� o  ���6�6 0 refnum RefNum�7  �9   � R      �5�4�3
�5 .ascrerr ****      � ****�4  �3   � k  �� ��� I �2��1
�2 .sysodlogaskr        TEXT� m  �� ���  k e i n   d a t u m . c�1  � ��0� I �/��.
�/ .rdwrclosnull���     ****� o  �-�- 0 refnum RefNum�.  �0   � ��� l �,�+�*�,  �+  �*  � ��� l �)���)  �   Neue Version einsetzen   � ��� .   N e u e   V e r s i o n   e i n s e t z e n� ��� r  ��� m  �� ���  � o      �(�( 0 filecontents fileContents� ��� r  *��� 4  (�'�
�' 
alis� l  '��&�%� l  '��$�#� I  '�"��
�" .earsffdralis        afdr�  f   !� �!�� 
�! 
rtyp� m  "#�
� 
ctxt�   �$  �#  �&  �%  � o      �� 0 
homeordner  � ��� l ++����  � 0 *display dialog "homeordner: " & homeordner   � ��� T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r� ��� r  +0��� n  +.��� m  ,.�
� 
ctnr� o  +,�� 0 
homeordner  � o      �� 0 homeordnerpfad  � ��� r  1:��� n  16��� 1  26�
� 
pnam� o  12�� 0 homeordnerpfad  � o      �� 0 projektname Projektname� ��� l ;;����  � 2 ,display dialog "Projektname: " & Projektname   � ��� X d i s p l a y   d i a l o g   " P r o j e k t n a m e :   "   &   P r o j e k t n a m e� ��� r  ;F��� n ;B��� 1  >B�
� 
txdl� 1  ;>�
� 
ascr� o      �� 0 olddels oldDels� ��� r  GR��� m  GJ�� ���  _� n     ��� 1  MQ�
� 
txdl� 1  JM�
� 
ascr� ��� l SS����  �  �  � ��� r  S^��� n  SZ��� 2 VZ�
� 
citm� o  SV�� 0 projektname Projektname� o      �� 0 zeilenliste Zeilenliste� ��� r  _j��� n  _f��� m  bf�

�
 
nmbr� o  _b�	�	 0 zeilenliste Zeilenliste� o      �� 0 	anzzeilen 	anzZeilen� ��� l kk����  � n hdisplay dialog "Zeilenliste: " & return & (Zeilenliste as list) & return & "Anzahl Zeilen: " & anzZeilen   � �   � d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   ( Z e i l e n l i s t e   a s   l i s t )   &   r e t u r n   &   " A n z a h l   Z e i l e n :   "   &   a n z Z e i l e n�  l kk����  �  �    l kk��   � �display dialog "Zeilenliste: " & return & item 1 of Zeilenliste & return & item 2 of Zeilenliste & return & item 3 of Zeilenliste & return & item 4 of Zeilenliste & return & item 5 of Zeilenliste    �� d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   i t e m   1   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   2   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   3   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   4   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   5   o f   Z e i l e n l i s t e 	 r  ky

 n  ku 4  nu�
� 
cobj l ot��  \  ot o  or���� 0 	anzzeilen 	anzZeilen m  rs���� �  �    o  kn���� 0 zeilenliste Zeilenliste o      ���� 0 version1 Version1	  r  z� n  z� 4  }���
�� 
cobj o  ~����� 0 	anzzeilen 	anzZeilen o  z}���� 0 zeilenliste Zeilenliste o      ���� 0 version2 Version2  r  �� o  ������ 0 olddels oldDels n      1  ����
�� 
txdl 1  ����
�� 
ascr   l ����������  ��  ��    !"! l ����#$��  # 2 ,set main to file "datum.c" of homeordnerpfad   $ �%% X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d" &'& r  ��()( b  ��*+* l ��,����, c  ��-.- o  ������ 0 homeordnerpfad  . m  ����
�� 
TEXT��  ��  + m  ��// �00  v e r s i o n . c) o      ���� 0 filepfad  ' 121 l ����34��  3 , &display dialog "filepfad: " & filepfad   4 �55 L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d2 676 I ��������
�� .miscactvnull��� ��� obj ��  ��  7 898 r  ��:;: l ��<����< I ����=>
�� .rdwropenshor       file= 4  ����?
�� 
file? o  ������ 0 filepfad  > ��@��
�� 
perm@ m  ����
�� boovtrue��  ��  ��  ; o      ���� 0 refnum RefNum9 ABA Q  �ICDEC k  �:FF GHG r  ��IJI l ��K����K I ����L��
�� .rdwrread****        ****L o  ������ 0 refnum RefNum��  ��  ��  J o      ���� 0 filecontents fileContentsH MNM l ����OP��  O 7 1display dialog "inhalt: " & return & fileContents   P �QQ b d i s p l a y   d i a l o g   " i n h a l t :   "   &   r e t u r n   &   f i l e C o n t e n t sN RSR l ����������  ��  ��  S TUT r  ��VWV n  ��XYX 4  ����Z
�� 
cparZ m  ������ Y o  ������ 0 filecontents fileContentsW o      ���� 0 alteversion  U [\[ l ����]^��  ] . (display dialog "Version: " & alteversion   ^ �__ P d i s p l a y   d i a l o g   " V e r s i o n :   "   &   a l t e v e r s i o n\ `a` r  ��bcb n  ��ded m  ����
�� 
nmbre n  ��fgf 2 ����
�� 
cha g o  ������ 0 alteversion  c o      ���� 0 l  a hih l ��jklj r  ��mnm n  ��opo 7 ����qr
�� 
ctxtq m  ������ r m  ������ p o  ������ 0 alteversion  n o      ���� 0 neueversion neueVersionk $  Anfang bis und mit Leerschlag   l �ss <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a gi tut l ����������  ��  ��  u vwv r  �xyx b  �z{z b  �|}| b  �~~ b  ���� b  ���� b  � ��� b  ����� n  ����� 4  �����
�� 
cpar� m  ������ � o  ������ 0 filecontents fileContents� o  ����
�� 
ret � o  ������ 0 neueversion neueVersion� m   �� ���  "� o  ���� 0 version1 Version1 m  �� ���  .} o  ���� 0 version2 Version2{ m  �� ���  "y o      ���� 0 	neuertext 	neuerTextw ��� l ������  � 4 .set paragraph 2 of fileContents to neueVersion   � ��� \ s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e V e r s i o n� ��� l ������  � : 4display dialog "neue Version: " & return & neuerText   � ��� h d i s p l a y   d i a l o g   " n e u e   V e r s i o n :   "   &   r e t u r n   &   n e u e r T e x t� ��� I $����
�� .rdwrseofnull���     ****� o  ���� 0 refnum RefNum� �����
�� 
set2� m   ����  ��  � ��� I %2����
�� .rdwrwritnull���     ****� o  %(���� 0 	neuertext 	neuerText� �����
�� 
refn� o  +.���� 0 refnum RefNum��  � ���� I 3:�����
�� .rdwrclosnull���     ****� o  36���� 0 refnum RefNum��  ��  D R      ������
�� .ascrerr ****      � ****��  ��  E k  BI�� ��� l BB��������  ��  ��  � ���� I BI�����
�� .rdwrclosnull���     ****� o  BE���� 0 refnum RefNum��  ��  B ��� l JJ��������  ��  ��  � ��� n  JO��� I  KO�������� $0 logaktualisieren LogAktualisieren��  ��  �  f  JK� ��� l PP��������  ��  ��  � ���� I P\�����
�� .aevtodocnull  �    alis� n  PX��� 4  QX���
�� 
file� m  TW�� ��� ( T W I _ M a s t e r . x c o d e p r o j� o  PQ���� 0 homeordnerpfad  ��  ��   	 m     ���                                                                                  MACS  alis    r  Macintosh HD               �� �H+   �:
Finder.app                                                      Ƙh        ����  	                CoreServices    ǿ�      ƘK�     �:  ��  ��  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��    ��� l     ��������  ��  ��  � ���� i     ��� I      �������� $0 logaktualisieren LogAktualisieren��  ��  � O    ���� k   ��� ��� I   	������
�� .miscactvnull��� ��� obj ��  ��  � ��� l  
 
��������  ��  ��  � ��� r   
 ��� m   
 �� ���  � o      ���� 0 filecontents fileContents� ��� r    ��� 4    ���
�� 
alis� l   ������ l   ������ I   ����
�� .earsffdralis        afdr�  f    � ���~
� 
rtyp� m    �}
�} 
ctxt�~  ��  ��  ��  ��  � o      �|�| 0 
homeordner  � ��� l   �{���{  � 0 *display dialog "homeordner: " & homeordner   � ��� T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r� ��� r     ��� n    ��� m    �z
�z 
ctnr� o    �y�y 0 
homeordner  � o      �x�x 0 homeordnerpfad  � ��� l  ! !�w���w  �  open homeordnerpfad   � ��� & o p e n   h o m e o r d n e r p f a d� ��� l  ! !�v���v  � 8 2display dialog "homeordnerpfad: " & homeordnerpfad   � ��� d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d� ��� l  ! !�u���u  � 2 ,set main to file "datum.c" of homeordnerpfad   � ��� X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d� ��� r   ! (��� b   ! &��� l  ! $��t�s� c   ! $��� o   ! "�r�r 0 homeordnerpfad  � m   " #�q
�q 
TEXT�t  �s  � m   $ %�� ���  L o g f i l e . t x t� o      �p�p 0 filepfad  � ��� l  ) )�o� �o  � , &display dialog "filepfad: " & filepfad     � L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d�  l  ) )�n�n   ! tell application "TextEdit"    � 6 t e l l   a p p l i c a t i o n   " T e x t E d i t "  I  ) .�m�l�k
�m .miscactvnull��� ��� obj �l  �k   	
	 r   / ; l  / 9�j�i I  / 9�h
�h .rdwropenshor       file 4   / 3�g
�g 
file o   1 2�f�f 0 filepfad   �e�d
�e 
perm m   4 5�c
�c boovtrue�d  �j  �i   o      �b�b 0 refnum RefNum
  Q   <` k   ?S  r   ? F l  ? D�a�` I  ? D�_�^
�_ .rdwrread****        **** o   ? @�]�] 0 refnum RefNum�^  �a  �`   o      �\�\ 0 filecontents fileContents  r   G P !  n   G N"#" 4  K N�[$
�[ 
cwor$ m   L M�Z�Z��# l  G K%�Y�X% n   G K&'& 4   H K�W(
�W 
cpar( m   I J�V�V ' o   G H�U�U 0 filecontents fileContents�Y  �X  ! o      �T�T 0 	lastdatum 	lastDatum )*) l  Q Q�S+,�S  + 7 1display dialog "lastDatum: " & return & lastDatum   , �-- b d i s p l a y   d i a l o g   " l a s t D a t u m :   "   &   r e t u r n   &   l a s t D a t u m* ./. l  Q Q�R01�R  0 . (set Datum to paragraph 2 of fileContents   1 �22 P s e t   D a t u m   t o   p a r a g r a p h   2   o f   f i l e C o n t e n t s/ 343 l  Q Q�Q56�Q  5 &  display dialog "Datum: " & Datum   6 �77 @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u m4 898 r   Q X:;: I  Q V�P�O�N
�P .misccurdldt    ��� null�O  �N  ; o      �M�M 	0 heute  9 <=< l  Y Y�L>?�L  > &  display dialog "heute: " & heute   ? �@@ @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e= ABA r   Y `CDC n   Y ^EFE 1   Z ^�K
�K 
yearF o   Y Z�J�J 	0 heute  D o      �I�I 0 jahrtext  B GHG r   a hIJI n   a fKLK m   b f�H
�H 
mnthL o   a b�G�G 	0 heute  J o      �F�F 0 	monattext  H MNM l  i i�EOP�E  O * $display dialog "monat: " & monattext   P �QQ H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x tN RSR r   i �TUT n   i ~VWV 7  r ~�DXY
�D 
ctxtX m   v z�C�C��Y m   { }�B�B��W l  i rZ�A�@Z b   i r[\[ m   i l]] �^^  0\ n   l q_`_ 1   m q�?
�? 
day ` o   l m�>�> 	0 heute  �A  �@  U o      �=�= 0 tag  S aba l  � ��<cd�<  c " display dialog "tag: " & tag   d �ee 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a gb fgf r   � �hih J   � �jj klk m   � ��;
�; 
jan l mnm m   � ��:
�: 
feb n opo m   � ��9
�9 
mar p qrq l 	 � �s�8�7s m   � ��6
�6 
apr �8  �7  r tut m   � ��5
�5 
may u vwv m   � ��4
�4 
jun w xyx m   � ��3
�3 
jul y z{z m   � ��2
�2 
aug { |}| l 	 � �~�1�0~ m   � ��/
�/ 
sep �1  �0  } � m   � ��.
�. 
oct � ��� m   � ��-
�- 
nov � ��,� m   � ��+
�+ 
dec �,  i o      �*�* 0 monatsliste MonatsListeg ��� Y   � ���)���(� Z   � ����'�&� =   � ���� o   � ��%�% 0 	monattext  � n   � ���� 4   � ��$�
�$ 
cobj� o   � ��#�# 0 i  � o   � ��"�" 0 monatsliste MonatsListe� k   � ��� ��� r   � ���� n   � ���� 7  � ��!��
�! 
ctxt� m   � �� � ��� m   � ������ l  � ����� b   � ���� m   � ��� ���  0� o   � ��� 0 i  �  �  � o      �� 	0 monat  � ��� l  � �����  S   � �� - ' wenn true, wird die Schleife verlassen   � ��� N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n�  �'  �&  �) 0 i  � m   � ��� � m   � ��� �(  � ��� l  � �����  � &  display dialog "monat: " & monat   � ��� @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t� ��� r   � ���� l 	 � ����� l  � ����� n  � ���� 7  � ����
� 
cha � m   � ��� � m   � ��� � l  � ����� c   � ���� o   � ��� 0 jahrtext  � m   � ��
� 
ctxt�  �  �  �  �  �  � o      �� 0 jahr  � ��� l  � ��
���
  � ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext   � ��� r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t� ��� l  � ��	���	  � , &set l to number of characters of Datum   � ��� L s e t   l   t o   n u m b e r   o f   c h a r a c t e r s   o f   D a t u m� ��� l  � �����  � 1 +set neuesDatum to text -l thru -13 of Datum   � ��� V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m� ��� l  � �����  � P Jset neuesDatum to text 1 thru 14 of Datum -- Anfang bis und mit Leerschlag   � ��� � s e t   n e u e s D a t u m   t o   t e x t   1   t h r u   1 4   o f   D a t u m   - -   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g� ��� r   ���� b   ���� b   ���� b   � ���� b   � ���� o   � ��� 0 tag  � m   � ��� ���  .� o   � ��� 	0 monat  � m   � �� ���  .� o  �� 0 jahrtext  � o      �� 0 
neuesdatum 
neuesDatum� ��� l ����  � 0 *display dialog "neuesDatum: " & neuesDatum   � ��� T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m� ��� Z  M����� = 	��� o  � �  0 
neuesdatum 
neuesDatum� o  ���� 0 	lastdatum 	lastDatum� l ������  � % display dialog "gleiches Datum"   � ��� > d i s p l a y   d i a l o g   " g l e i c h e s   D a t u m "�  � k  M�� ��� l ��������  ��  ��  � ��� r  9��� b  7��� b  3��� b  1��� b  -��� b  )��� b  %��� b  !��� b  ��� b  � � b   m   � T * * * * * * * * * * * * * * * * * * * * * *                                         o  ���� 0 
neuesdatum 
neuesDatum  o  ��
�� 
ret � l 	���� o  ��
�� 
ret ��  ��  � o   ��
�� 
ret � o  !$��
�� 
ret � o  %(��
�� 
ret � l 	),���� m  ), � , * * * * * * * * * * * * * * * * * * * * * *��  ��  � o  -0��
�� 
ret � o  12���� 0 filecontents fileContents� o  36��
�� 
ret � o      ���� 0 	neuertext 	neuerText� 	
	 I :C��
�� .rdwrseofnull���     **** o  :;���� 0 refnum RefNum ����
�� 
set2 m  >?����  ��  
 �� I DM��
�� .rdwrwritnull���     **** o  DE���� 0 	neuertext 	neuerText ����
�� 
refn o  HI���� 0 refnum RefNum��  ��  � �� I NS����
�� .rdwrclosnull���     **** o  NO���� 0 refnum RefNum��  ��   R      ������
�� .ascrerr ****      � ****��  ��   I [`����
�� .rdwrclosnull���     **** o  [\���� 0 refnum RefNum��    l aa����    start    � 
 s t a r t  r  aj J  af �� m  ad   �!!  x c o d e p r o j��   o      ���� 0 filetype   "#" l kk��$%��  $ ? 9set projektpfad to (path to alias (homeordner)) as string   % �&& r s e t   p r o j e k t p f a d   t o   ( p a t h   t o   a l i a s   ( h o m e o r d n e r ) )   a s   s t r i n g# '(' l kk��)*��  ) 0 *display dialog "projektpfad" & projektpfad   * �++ T d i s p l a y   d i a l o g   " p r o j e k t p f a d "   &   p r o j e k t p f a d( ,-, l kk��./��  . 8 2display dialog "homeordnerpfad: " & homeordnerpfad   / �00 d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d- 121 l kk��34��  3 > 8get name of folders of folder (homeordnerpfad as string)   4 �55 p g e t   n a m e   o f   f o l d e r s   o f   f o l d e r   ( h o m e o r d n e r p f a d   a s   s t r i n g )2 676 l k}89:8 r  k};<; n  ky=>= 1  uy��
�� 
pnam> n  ku?@? 2 su��
�� 
file@ 4  ks��A
�� 
cfolA l orB����B c  orCDC o  op���� 0 homeordnerpfad  D m  pq��
�� 
TEXT��  ��  < o      ���� 
0 inhalt  9  without invisibles   : �EE $ w i t h o u t   i n v i s i b l e s7 FGF l ~~��HI��  H # display dialog inhalt as text   I �JJ : d i s p l a y   d i a l o g   i n h a l t   a s   t e x tG KLK l ~~��MN��  M 7 1repeat with i from 1 to number of items of inhalt   N �OO b r e p e a t   w i t h   i   f r o m   1   t o   n u m b e r   o f   i t e m s   o f   i n h a l tL P��P X  ~�Q��RQ k  ��SS TUT l ����VW��  V &  display dialog (dasFile) as text   W �XX @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x tU Y��Y Z  ��Z[����Z E  ��\]\ l ��^����^ l ��_����_ o  ������ 0 dasfile dasFile��  ��  ��  ��  ] m  ��`` �aa  x c o d e p r o j[ k  ��bb cdc r  ��efe b  ��ghg l ��i����i c  ��jkj o  ������ 0 homeordnerpfad  k m  ����
�� 
ctxt��  ��  h l ��l����l c  ��mnm o  ������ 0 dasfile dasFilen m  ����
�� 
ctxt��  ��  f o      ���� 0 filepfad  d opo l ����qr��  q &  display dialog (dasFile) as text   r �ss @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x tp t��t I ����u��
�� .aevtodocnull  �    alisu 4  ����v
�� 
filev o  ������ 0 filepfad  ��  ��  ��  ��  ��  �� 0 dasfile dasFileR o  ������ 
0 inhalt  ��  � m     ww�                                                                                  MACS  alis    r  Macintosh HD               �� �H+   �:
Finder.app                                                      Ƙh        ����  	                CoreServices    ǿ�      ƘK�     �:  ��  ��  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��       ��xyz��  x ������ $0 logaktualisieren LogAktualisieren
�� .aevtoappnull  �   � ****y �������{|���� $0 logaktualisieren LogAktualisieren��  ��  { ���������������������������������������� 0 filecontents fileContents�� 0 
homeordner  �� 0 homeordnerpfad  �� 0 filepfad  �� 0 refnum RefNum�� 0 	lastdatum 	lastDatum�� 	0 heute  �� 0 jahrtext  �� 0 	monattext  �� 0 tag  �� 0 monatsliste MonatsListe�� 0 i  �� 	0 monat  �� 0 jahr  �� 0 
neuesdatum 
neuesDatum�� 0 	neuertext 	neuerText�� 0 filetype  �� 
0 inhalt  �� 0 dasfile dasFile| :w����������������������������������]������~�}�|�{�z�y�x�w�v�u�t�s�r��q�p���o�n�m�l�k�j�i�h �g�f�e�d`�c
�� .miscactvnull��� ��� obj 
�� 
alis
�� 
rtyp
�� 
ctxt
�� .earsffdralis        afdr
�� 
ctnr
�� 
TEXT
�� 
file
�� 
perm
�� .rdwropenshor       file
�� .rdwrread****        ****
�� 
cpar
�� 
cwor
�� .misccurdldt    ��� null
�� 
year
�� 
mnth
�� 
day ����
� 
jan 
�~ 
feb 
�} 
mar 
�| 
apr 
�{ 
may 
�z 
jun 
�y 
jul 
�x 
aug 
�w 
sep 
�v 
oct 
�u 
nov 
�t 
dec �s 
�r 
cobj
�q 
cha �p 
�o 
ret 
�n 
set2
�m .rdwrseofnull���     ****
�l 
refn
�k .rdwrwritnull���     ****
�j .rdwrclosnull���     ****�i  �h  
�g 
cfol
�f 
pnam
�e 
kocl
�d .corecnte****       ****
�c .aevtodocnull  �    alis�����*j O�E�O*�)��l /E�O��,E�O��&�%E�O*j O*�/�el E�O�j E�O��k/�i/E�O*j E�O�a ,E�O�a ,E�Oa �a ,%[�\[Za \Zi2E�Oa a a a a a a a a a a  a !a "vE�O 4ka "kh ��a #�/  a $�%[�\[Za \Zi2E�OY h[OY��O��&[a %\[Zm\Za &2E�O�a '%�%a (%�%E�O��  hY ?a )�%_ *%_ *%_ *%_ *%_ *%a +%_ *%�%_ *%E�O�a ,jl -O�a .�l /O�j 0W X 1 2�j 0Oa 3kvE^ O*a 4��&/�-a 5,E^ O <] [a 6a #l 7kh ] a 8 ��&] �&%E�O*�/j 9Y h[OY��Uz �b}�a�`~�_
�b .aevtoappnull  �   � ****} k    ]��  �^�^  �a  �`  ~ �]�] 0 i   g� �\�[�Z�Y�X�W ,�V�U�T�S�R K�Q R�P�O�N�M�L�K�J ��I�H�G�F�E�D�C�B�A�@�?�>�=�<�;�:�9 ��8�7�6�5�4�3�2�1�0 ��/ "$&-6�.�-�,�+�*�)�(|~���'�&�%�$�#�"�!��� ����������/����������\ 0 filecontents fileContents
�[ 
alis
�Z 
rtyp
�Y 
ctxt
�X .earsffdralis        afdr�W 0 
homeordner  
�V .sysodlogaskr        TEXT
�U 
cobj
�T 
ctnr�S 0 homeordnerpfad  
�R 
TEXT�Q 0 filepfad  
�P .miscactvnull��� ��� obj 
�O .misccurdldt    ��� null�N 	0 heute  
�M 
year�L 0 jahrtext  
�K 
mnth�J 0 	monattext  
�I 
day �H���G 0 tag  
�F 
jan 
�E 
feb 
�D 
mar 
�C 
apr 
�B 
may 
�A 
jun 
�@ 
jul 
�? 
aug 
�> 
sep 
�= 
oct 
�< 
nov 
�; 
dec �: �9 0 monatsliste MonatsListe�8 	0 monat  
�7 
cha �6 �5 0 jahr  
�4 
file
�3 
perm
�2 .rdwropenshor       file�1 0 refnum RefNum
�0 .rdwrread****        ****�/ 0 
neuesdatum 
neuesDatum
�. 
ret �- 0 	neuertext 	neuerText
�, 
cpar�+ 0 datum Datum
�* 
nmbr�) 0 l  �( 
�' 
set2
�& .rdwrseofnull���     ****
�% 
refn
�$ .rdwrwritnull���     ****
�# .rdwrclosnull���     ****�"  �!  
�  
pnam� 0 projektname Projektname
� 
ascr
� 
txdl� 0 olddels oldDels
� 
citm� 0 zeilenliste Zeilenliste� 0 	anzzeilen 	anzZeilen� 0 version1 Version1� 0 version2 Version2� 0 alteversion  � � 0 neueversion neueVersion� $0 logaktualisieren LogAktualisieren
� .aevtodocnull  �    alis�_^�Z�E�O*�)��l /E�O��%j 	O*��/�,�&E�O��&�%E�Oa �%j 	O*j O*j E` O_ a ,E` O_ a ,E` Oa _ a ,%[�\[Za \Zi2E` Oa a a a a  a !a "a #a $a %a &a 'a (vE` )O 8ka (kh  _ _ )�/  a *�%[�\[Za \Zi2E` +OY h[OY��O_ �&[a ,\[Zm\Za -2E` .O*a /�/a 0el 1E` 2O �_ 2j 3E�O�a 4  Ta 5j 	Oa 6E` 7O_ 7a 8%_ %a 9%_ +%a :%_ %a ;%E` 7Oa <_ 7%j 	Oa =_ >%_ 7%E` ?Y i�a @l/E` AO_ Aa ,-a B,E` CO_ A[�\[Zk\Za D2E` 7O_ 7a E%_ %a F%_ +%a G%_ %a H%E` 7O�a @k/_ >%_ 7%E` ?OPO_ 2a Ijl JO_ ?a K_ 2l LO_ 2j MW X N Oa Pj 	O_ 2j MOa QE�O*�)��l /E�O��,E�O�a R,E` SO_ Ta U,E` VOa W_ Ta U,FO_ Sa X-E` YO_ Ya B,E` ZO_ Y�_ Zk/E` [O_ Y�_ Z/E` \O_ V_ Ta U,FO��&a ]%E�O*j O*a /�/a 0el 1E` 2O �_ 2j 3E�O�a @l/E` ^O_ ^a ,-a B,E` CO_ ^[�\[Zk\Za _2E` `O�a @k/_ >%_ `%a a%_ [%a b%_ \%a c%E` ?O_ 2a Ijl JO_ ?a K_ 2l LO_ 2j MW X N O_ 2j MO)j+ dO�a /a e/j fU ascr  ��ޭ