FasdUAS 1.101.10   ��   ��    k             l   � ����  O    �  	  k   � 
 
     I   	������
�� .miscactvnull��� ��� obj ��  ��        l  
 
��������  ��  ��        r   
     m   
    �      o      ���� 0 filecontents fileContents      r        4    �� 
�� 
alis  l    ����  l    ����  I   ��  
�� .earsffdralis        afdr   f      �� ��
�� 
rtyp  m    ��
�� 
ctxt��  ��  ��  ��  ��    o      ���� 0 
homeordner         l   �� ! "��   ! 0 *display dialog "homeordner: " & homeordner    " � # # T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r    $ % $ r      & ' & n     ( ) ( m    ��
�� 
ctnr ) o    ���� 0 
homeordner   ' o      ���� 0 homeordnerpfad   %  * + * l  ! !�� , -��   ,  open homeordnerpfad    - � . . & o p e n   h o m e o r d n e r p f a d +  / 0 / l  ! !�� 1 2��   1 8 2display dialog "homeordnerpfad: " & homeordnerpfad    2 � 3 3 d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d 0  4 5 4 l  ! !�� 6 7��   6 2 ,set main to file "datum.c" of homeordnerpfad    7 � 8 8 X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d 5  9 : 9 r   ! ( ; < ; b   ! & = > = l  ! $ ?���� ? c   ! $ @ A @ o   ! "���� 0 homeordnerpfad   A m   " #��
�� 
TEXT��  ��   > m   $ % B B � C C  L o g f i l e . t x t < o      ���� 0 filepfad   :  D E D l  ) )�� F G��   F , &display dialog "filepfad: " & filepfad    G � H H L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d E  I J I l  ) )�� K L��   K ! tell application "TextEdit"    L � M M 6 t e l l   a p p l i c a t i o n   " T e x t E d i t " J  N O N I  ) .������
�� .miscactvnull��� ��� obj ��  ��   O  P Q P r   / = R S R l  / 9 T���� T I  / 9�� U V
�� .rdwropenshor       file U 4   / 3�� W
�� 
file W o   1 2���� 0 filepfad   V �� X��
�� 
perm X m   4 5��
�� boovtrue��  ��  ��   S o      ���� 0 refnum RefNum Q  Y Z Y Q   >� [ \ ] [ k   A� ^ ^  _ ` _ r   A J a b a l  A H c���� c I  A H�� d��
�� .rdwrread****        **** d o   A D���� 0 refnum RefNum��  ��  ��   b o      ���� 0 filecontents fileContents `  e f e r   K Z g h g n   K V i j i 4  Q V�� k
�� 
cwor k m   T U������ j l  K Q l���� l n   K Q m n m 4   L Q�� o
�� 
cpar o m   O P����  n o   K L���� 0 filecontents fileContents��  ��   h o      ���� 0 	lastdatum 	lastDatum f  p q p l  [ [�� r s��   r 7 1display dialog "lastDatum: " & return & lastDatum    s � t t b d i s p l a y   d i a l o g   " l a s t D a t u m :   "   &   r e t u r n   &   l a s t D a t u m q  u v u l  [ [�� w x��   w . (set Datum to paragraph 2 of fileContents    x � y y P s e t   D a t u m   t o   p a r a g r a p h   2   o f   f i l e C o n t e n t s v  z { z l  [ [�� | }��   | &  display dialog "Datum: " & Datum    } � ~ ~ @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u m {   �  r   [ d � � � I  [ `������
�� .misccurdldt    ��� null��  ��   � o      ���� 	0 heute   �  � � � l  e e�� � ���   � &  display dialog "heute: " & heute    � � � � @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e �  � � � r   e p � � � n   e l � � � 1   h l��
�� 
year � o   e h���� 	0 heute   � o      ���� 0 jahrtext   �  � � � r   q | � � � n   q x � � � m   t x��
�� 
mnth � o   q t���� 	0 heute   � o      ���� 0 	monattext   �  � � � l  } }�� � ���   � * $display dialog "monat: " & monattext    � � � � H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x t �  � � � r   } � � � � n   } � � � � 7  � ��� � �
�� 
ctxt � m   � ������� � m   � ������� � l  } � ����� � b   } � � � � m   } � � � � � �  0 � n   � � � � � 1   � ���
�� 
day  � o   � ����� 	0 heute  ��  ��   � o      ���� 0 tag   �  � � � l  � ��� � ���   � " display dialog "tag: " & tag    � � � � 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g �  � � � r   � � � � � J   � � � �  � � � m   � ���
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
dec ��   � o      ���� 0 monatsliste MonatsListe �  � � � Y   � ��� � ��� � Z   � � � ����� � =   � � � � � o   � ����� 0 	monattext   � n   � � � � � 4   � ��� �
�� 
cobj � o   � ����� 0 i   � o   � ����� 0 monatsliste MonatsListe � k   � � � �  � � � r   � � � � � n   � � � � � 7  � ��� � �
�� 
ctxt � m   � ������� � m   � ������� � l  � � ����� � b   � � � � � m   � � � � � � �  0 � o   � ����� 0 i  ��  ��   � o      ���� 	0 monat   �  ��� � l  � � � � � �  S   � � � - ' wenn true, wird die Schleife verlassen    � � � � N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n��  ��  ��  �� 0 i   � m   � �����  � m   � ����� ��   �  � � � l �� � ���   � &  display dialog "monat: " & monat    � � � � @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t �  � � � r   � � � l 	 ����� � l  ����� � n  � � � 7 �� � �
�� 
cha  � m  ����  � m  ����  � l  ���~ � c   � � � o  �}�} 0 jahrtext   � m  �|
�| 
ctxt�  �~  ��  ��  ��  ��   � o      �{�{ 0 jahr   �  � � � l �z � ��z   � ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext    � �   r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t �  l �y�y   , &set l to number of characters of Datum    � L s e t   l   t o   n u m b e r   o f   c h a r a c t e r s   o f   D a t u m  l �x	�x   1 +set neuesDatum to text -l thru -13 of Datum   	 �

 V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m  l �w�w   P Jset neuesDatum to text 1 thru 14 of Datum -- Anfang bis und mit Leerschlag    � � s e t   n e u e s D a t u m   t o   t e x t   1   t h r u   1 4   o f   D a t u m   - -   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g  r  1 b  - b  ) b  % b  ! o  �v�v 0 tag   m    �  . o  !$�u�u 	0 monat   m  %( �  . o  ),�t�t 0 jahrtext   o      �s�s 0 
neuesdatum 
neuesDatum  !  l 22�r"#�r  " 0 *display dialog "neuesDatum: " & neuesDatum   # �$$ T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m! %&% Z  2�'(�q)' = 29*+* o  25�p�p 0 
neuesdatum 
neuesDatum+ o  58�o�o 0 	lastdatum 	lastDatum( l <<�n,-�n  , % display dialog "gleiches Datum"   - �.. > d i s p l a y   d i a l o g   " g l e i c h e s   D a t u m "�q  ) k  @�// 010 l @@�m�l�k�m  �l  �k  1 232 r  @m454 b  @i676 b  @e898 b  @c:;: b  @_<=< b  @[>?> b  @W@A@ b  @SBCB b  @ODED b  @KFGF b  @GHIH m  @CJJ �KK T * * * * * * * * * * * * * * * * * * * * * *                                        I o  CF�j�j 0 
neuesdatum 
neuesDatumG o  GJ�i
�i 
ret E l 	KNL�h�gL o  KN�f
�f 
ret �h  �g  C o  OR�e
�e 
ret A o  SV�d
�d 
ret ? o  WZ�c
�c 
ret = l 	[^M�b�aM m  [^NN �OO , * * * * * * * * * * * * * * * * * * * * * *�b  �a  ; o  _b�`
�` 
ret 9 o  cd�_�_ 0 filecontents fileContents7 o  eh�^
�^ 
ret 5 o      �]�] 0 	neuertext 	neuerText3 PQP I ny�\RS
�\ .rdwrseofnull���     ****R o  nq�[�[ 0 refnum RefNumS �ZT�Y
�Z 
set2T m  tu�X�X  �Y  Q U�WU I z��VVW
�V .rdwrwritnull���     ****V o  z}�U�U 0 	neuertext 	neuerTextW �TX�S
�T 
refnX o  ���R�R 0 refnum RefNum�S  �W  & Y�QY I ���PZ�O
�P .rdwrclosnull���     ****Z o  ���N�N 0 refnum RefNum�O  �Q   \ R      �M�L�K
�M .ascrerr ****      � ****�L  �K   ] I ���J[�I
�J .rdwrclosnull���     ****[ o  ���H�H 0 refnum RefNum�I   Z \]\ l ���G^_�G  ^  start   _ �`` 
 s t a r t] aba r  ��cdc J  ��ee f�Ff m  ��gg �hh  x c o d e p r o j�F  d o      �E�E 0 filetype  b iji l ���Dkl�D  k ? 9set projektpfad to (path to alias (homeordner)) as string   l �mm r s e t   p r o j e k t p f a d   t o   ( p a t h   t o   a l i a s   ( h o m e o r d n e r ) )   a s   s t r i n gj non l ���Cpq�C  p 0 *display dialog "projektpfad" & projektpfad   q �rr T d i s p l a y   d i a l o g   " p r o j e k t p f a d "   &   p r o j e k t p f a do sts l ���Buv�B  u 8 2display dialog "homeordnerpfad: " & homeordnerpfad   v �ww d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a dt xyx l ���Az{�A  z > 8get name of folders of folder (homeordnerpfad as string)   { �|| p g e t   n a m e   o f   f o l d e r s   o f   f o l d e r   ( h o m e o r d n e r p f a d   a s   s t r i n g )y }~} l ���� r  ����� n  ����� 1  ���@
�@ 
pnam� n  ����� 2 ���?
�? 
file� 4  ���>�
�> 
cfol� l ����=�<� c  ����� o  ���;�; 0 homeordnerpfad  � m  ���:
�: 
TEXT�=  �<  � o      �9�9 
0 inhalt  �  without invisibles   � ��� $ w i t h o u t   i n v i s i b l e s~ ��� l ���8���8  � # display dialog inhalt as text   � ��� : d i s p l a y   d i a l o g   i n h a l t   a s   t e x t� ��� l ���7���7  � 7 1repeat with i from 1 to number of items of inhalt   � ��� b r e p e a t   w i t h   i   f r o m   1   t o   n u m b e r   o f   i t e m s   o f   i n h a l t� ��6� X  ����5�� k  ���� ��� l ���4���4  � &  display dialog (dasFile) as text   � ��� @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x t� ��3� Z  �����2�1� E  ����� l ����0�/� l ����.�-� o  ���,�, 0 dasfile dasFile�.  �-  �0  �/  � m  ���� ���  x c o d e p r o j� k  ���� ��� r  ����� b  ����� l ����+�*� c  ����� o  ���)�) 0 homeordnerpfad  � m  ���(
�( 
ctxt�+  �*  � l ����'�&� c  ����� o  ���%�% 0 dasfile dasFile� m  ���$
�$ 
ctxt�'  �&  � o      �#�# 0 filepfad  � ��� l ���"���"  � &  display dialog (dasFile) as text   � ��� @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x t� ��!� I ��� ��
�  .aevtodocnull  �    alis� 4  ����
� 
file� o  ���� 0 filepfad  �  �!  �2  �1  �3  �5 0 dasfile dasFile� o  ���� 
0 inhalt  �6   	 m     ���                                                                                  MACS  alis    r  Macintosh HD               �� �H+   �:
Finder.app                                                      Ƙh        ����  	                CoreServices    ǿ�      ƘK�     �:  ��  ��  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��    ��� l     ����  �  �  �       ����  � �
� .aevtoappnull  �   � ****� �������
� .aevtoappnull  �   � ****� k    ���  ��  �  �  � ��� 0 i  � 0 dasfile dasFile� K�� ����
�	���� B����� �������������������� ������������������������������������� �����������J��N����������������g���������������
� .miscactvnull��� ��� obj � 0 filecontents fileContents
� 
alis
� 
rtyp
�
 
ctxt
�	 .earsffdralis        afdr� 0 
homeordner  
� 
ctnr� 0 homeordnerpfad  
� 
TEXT� 0 filepfad  
� 
file
� 
perm
� .rdwropenshor       file�  0 refnum RefNum
�� .rdwrread****        ****
�� 
cpar
�� 
cwor�� 0 	lastdatum 	lastDatum
�� .misccurdldt    ��� null�� 	0 heute  
�� 
year�� 0 jahrtext  
�� 
mnth�� 0 	monattext  
�� 
day ������ 0 tag  
�� 
jan 
�� 
feb 
�� 
mar 
�� 
apr 
�� 
may 
�� 
jun 
�� 
jul 
�� 
aug 
�� 
sep 
�� 
oct 
�� 
nov 
�� 
dec �� �� 0 monatsliste MonatsListe
�� 
cobj�� 	0 monat  
�� 
cha �� �� 0 jahr  �� 0 
neuesdatum 
neuesDatum
�� 
ret �� 0 	neuertext 	neuerText
�� 
set2
�� .rdwrseofnull���     ****
�� 
refn
�� .rdwrwritnull���     ****
�� .rdwrclosnull���     ****��  ��  �� 0 filetype  
�� 
cfol
�� 
pnam�� 
0 inhalt  
�� 
kocl
�� .corecnte****       ****
�� .aevtodocnull  �    alis����*j O�E�O*�)��l /E�O��,E�O��&�%E�O*j O*��/�el E` OS_ j E�O�a k/a i/E` O*j E` O_ a ,E` O_ a ,E` Oa _ a ,%[�\[Za \Zi2E` Oa  a !a "a #a $a %a &a 'a (a )a *a +a ,vE` -O :ka ,kh  _ _ -a .�/  a /�%[�\[Za \Zi2E` 0OY h[OY��O_ �&[a 1\[Zm\Za 22E` 3O_ a 4%_ 0%a 5%_ %E` 6O_ 6_   hY Ia 7_ 6%_ 8%_ 8%_ 8%_ 8%_ 8%a 9%_ 8%�%_ 8%E` :O_ a ;jl <O_ :a =_ l >O_ j ?W X @ A_ j ?Oa BkvE` CO*a D��&/�-a E,E` FO 8_ F[a Ga .l Hkh �a I ��&��&%E�O*��/j JY h[OY��Uascr  ��ޭ