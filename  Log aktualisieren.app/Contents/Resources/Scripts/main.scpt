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
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��    ��� l     ����  �  �  �       "������������������������������
�	����  �  ������ ����������������������������������������������������
� .aevtoappnull  �   � ****� 0 filecontents fileContents� 0 
homeordner  � 0 homeordnerpfad  � 0 filepfad  �  0 refnum RefNum�� 0 	lastdatum 	lastDatum�� 	0 heute  �� 0 jahrtext  �� 0 	monattext  �� 0 tag  �� 0 monatsliste MonatsListe�� 	0 monat  �� 0 jahr  �� 0 
neuesdatum 
neuesDatum�� 0 	neuertext 	neuerText�� 0 filetype  �� 
0 inhalt  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  � �����������
�� .aevtoappnull  �   � ****� k    ���  ����  ��  ��  � ������ 0 i  �� 0 dasfile dasFile� K��� ������������������ B������������������������������ ������������������������������������� �����������J��N����������������g���������������
�� .miscactvnull��� ��� obj �� 0 filecontents fileContents
�� 
alis
�� 
rtyp
�� 
ctxt
�� .earsffdralis        afdr�� 0 
homeordner  
�� 
ctnr�� 0 homeordnerpfad  
�� 
TEXT�� 0 filepfad  
�� 
file
�� 
perm
�� .rdwropenshor       file�� 0 refnum RefNum
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
�� .aevtodocnull  �    alis�����*j O�E�O*�)��l /E�O��,E�O��&�%E�O*j O*��/�el E` OS_ j E�O�a k/a i/E` O*j E` O_ a ,E` O_ a ,E` Oa _ a ,%[�\[Za \Zi2E` Oa  a !a "a #a $a %a &a 'a (a )a *a +a ,vE` -O :ka ,kh  _ _ -a .�/  a /�%[�\[Za \Zi2E` 0OY h[OY��O_ �&[a 1\[Zm\Za 22E` 3O_ a 4%_ 0%a 5%_ %E` 6O_ 6_   hY Ia 7_ 6%_ 8%_ 8%_ 8%_ 8%_ 8%a 9%_ 8%�%_ 8%E` :O_ a ;jl <O_ :a =_ l >O_ j ?W X @ A_ j ?Oa BkvE` CO*a D��&/�-a E,E` FO 8_ F[a Ga .l Hkh �a I ��&��&%E�O*��/j JY h[OY��U� ���N * * * * * * * * * * * * * * * * * * * * * *                                         1 8 . 0 7 . 2 0 1 0  l   3 3 9 2 :   B U S _ S t a t u s   &   ( 1 < < T W I _ C O N T R O L B I T ) ,   ( 1 < < W E B _ C O N T R O L B I T )   a u s   S t a t u s t a s k   e n t f e r n t  l   3 4 0 0 :   R u e c k s e t z e n   v o n   S t a t u s b i t   w i e d e r   e i n g e f � g t    * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         0 8 . 0 7 . 2 0 1 0  l   5 9 7 :   I S R :   ( s p i s t a t u s & ( 1 < < T W I _ S T A T U S B I T ) )   i n   A b f r a g e   e i n b e z o g e n     * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         0 5 . 0 7 . 2 0 1 0  l   3 3 5 7   B U S _ S t a t u s   & = ~ ( 1 < < T W I _ C O N T R O L B I T ) ;   g e l o e s c h t  l   1 5 4 1 :   s p i s t a t u s   & =   ~ ( 1 < < S P I _ S E N D B I T ) ;   v o r   i f   s c h r e i b s t a t u s   v e r s c h o b e n  l   3 5 1 5 :   s h i f t   n u r   w e n n   s e n d b i t   u n d   k e i n   e r r b i t  l   3 4 1 9 :   B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ; 	 	 / /   T W I   w i e d e r   O N 	   g e l o e s c h t ,   i n   S t a t u s t a s k - A b r a g e   v e s c h o b e n  * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         0 3 . 0 7 . 2 0 1 0  S t a t u s t a s k   a b f r a g e n ,   b e i   S t a t u s   0   t i m e r 0   d e a k t i v i e r e n :   K e i n e   S t o e r u n g e n   d u c h   S P I - A u f r u f e   d e s   T i m e r s     * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         3 0 . 0 6 . 2 0 1 0       * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         2 7 . 0 6 . 2 0 1 0  D a t e n   v o n   E s t r i c h   a u c h   a m   A n f a n g   v o n   s p i b u f f e r   e i n g e s e t z t      * * * * * * * * * * * * * * * * * * * * * *   * * * * * * * * * * * * * * * * * * * * * *  T W I _ M a s t e r 3 2 _ 9 0 _ S R _ 1 6   V e r s i o n :   S R _ 1 6  2 6 . 0 3 . 2 0 1 0   * *  2 1 . 3 . 1 0  D A T A T A S K   a u f   C 0   g e s e t z t   ( W e b T x S t a r t D a t e n   > =   D A T A T A S K )   a n s t a t t   = = ,   V o r b e r e i t u n g   f u e r   u n t e r s c h e i d e n   v o n   S O L A R T A S K   u n d   D A T A T A S K   * *   5 . 2 . 1 0  B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ;   K o n t r o l l i e r e n ,   w i e   T W I   w i e d e r   e i n g e s c h a l t e t   w i r d  L i n i e   1 6 0 0  T i m e r   f u e r   T W I   z u r u e c k s e t z e n   f u e r   E E P R O M W R I T E T A S K   T e s t :    B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ;   i n   I S R :   e v e n t u e l l   z u   f r u e h .     	 	  i n   T a l k ( )   u n d   L i s t e n ( ) :  / /   T i m e r   f u e r   T W I - S c h l a u f e   z u r u e c k s e t z e n  	 	 T C N T 0   =   0 x 0 0 ;  	 	 S I G N A L _ C o u n t = 0 ;    * * *  1 0 . 1 2 . 0 9  s e t R e a d y T o S e n d   v o n   B e d i n g u n g   D a t e n [ 2 , 3 , 4 ]   n i c h t   N u l l   a b h � n g i g   g e m a c h t   2 5 . 1 0 . 0 9  c o u n t   d u r c h   S I G N A L _ _ c o u n t   e r s e t z t   2 . 2 . 0 9  l i n e   2 2 1 5  E i n g e f � g t :    i 2 c _ s t o p ( ) ;  T W I   z u t u e c k s e t z e n ;  T W B R   = 0 ;  d e l a y _ m s ( 1 0 ) ;   / / 	 T W I   n e u   s t a r t e n  i 2 c _ i n i t ( ) ;    1 . 2 .  l i n e   8 7 3  A n f a n g s w e r t   v o n   s t a r t d e l a y   i n   O D E R   e i n g e f u e g t :   K o n t r o l l e   l a e f t   a u c h   s o  s t a r t d e l a y   = =     P i n s   f u e r   T W I   z u e r s t   a l s   A u s g a e n g e   u n d   H I  E r s t   n a c h   A b l a u f   S t a r t d e l a y   z u   E i n g a e n g e n   g e m a c h t   l i n e   1 0 4 6  i 2 c _ s t o p   e i n g e f u e g t   l i n e   1 4 2 4  E n d e   v o n   i f   ( D C F 7 7 - e r f o l g = = 0 )   n a c h   2 1 4 4   v e r s c h o b e n :   A b f r a g e   d e r   S l a v e s   n u r   w e n n   U h r   g e l e s e n   w e r d e n   k a n n     l i n e   9 1 6  T W B R   = 0 ;   e r s e t z t   d u r c h  T W C R   = 0 ;  Q u e l l e :   f i l e : / / / U s e r s / s y s a d m i n / D o c u m e n t s / E l e k t r o n i k / A V R / I 2 C / T W I - E r r o r / T W I % 2 0 - % 2 0 A V R % 2 0 s t � r z t % 2 0 a b % 2 0 - % 2 0 m i k r o c o n t r o l l e r . n e t . w e b a r c h i v e   B U S _ S t a t u s :     3 1 . 1 . 0 9  l i n e   2 0 7 0    i 2 c _ s t o p ( )   e i n g e s e t z t     T W I _ M a s t e r 3 2 _ 9 0 _ S R _ 1 6  * * * * * * * * * * * * *  2 6 . 0 3 . 2 0 1 0                                                                                 � �� ����� ����� ����� ����� ����� ����� ����� ����� ����� ����� ����� ���
�� 
sdsk
�� 
cfol� ��� 
 U s e r s
�� 
cfol� ���  s y s a d m i n
�� 
cfol� ���  D o c u m e n t s
�� 
cfol� ���  E l e k t r o n i k
�� 
cfol� ���    A V R
�� 
cfol� ���    A V R _ P r o g r a m m e
�� 
cfol� ���  A V R   W e b S e r v e r
�� 
cfol� ��� $   W e b S e r v e r   a k t u e l l
�� 
cfol� ���  W E B _ S P I _ 2
�� 
cfol� ��� & T W I _ M a s t e r 6 4 4 _ 2 0 _ 1 7
�� 
appf� ��� ,   L o g   a k t u a l i s i e r e n . a p p� �� ����� ����� ����� �����  ��  �� �� �� ��	 
��
 ���
�� 
sdsk
�� 
cfol � 
 U s e r s
�� 
cfol	 �  s y s a d m i n
�� 
cfol �  D o c u m e n t s
�� 
cfol �  E l e k t r o n i k
�� 
cfol �    A V R
�� 
cfol �    A V R _ P r o g r a m m e
�� 
cfol� �  A V R   W e b S e r v e r
�� 
cfol� � $   W e b S e r v e r   a k t u e l l
�� 
cfol� �  W E B _ S P I _ 2
�� 
cfol� � & T W I _ M a s t e r 6 4 4 _ 2 0 _ 1 7� �0 M a c i n t o s h   H D : U s e r s : s y s a d m i n : D o c u m e n t s : E l e k t r o n i k :   A V R :   A V R _ P r o g r a m m e : A V R   W e b S e r v e r :   W e b S e r v e r   a k t u e l l : W E B _ S P I _ 2 : T W I _ M a s t e r 6 4 4 _ 2 0 _ 1 7 : T W I _ M a s t e r . x c o d e p r o j� � �  1 8 . 0 7 . 2 0 1 0� ldt     �h����
� 
jul � �  1 8� ����   ����������������~�}�|�{
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
� 
aug 
�~ 
sep 
�} 
oct 
�| 
nov 
�{ 
dec � �  0 7� �z�z   �y�x�w�v�u�t�s�r�q�p�o�n�m�l �  1 �  0�y  �x  �w  �v  �u  �t  �s  �r  �q  �p  �o  �n  �m  �l  � �    1 8 . 0 7 . 2 0 1 0� �!! * * * * * * * * * * * * * * * * * * * * * *                                         0 8 . 0 7 . 2 0 1 0      * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         0 5 . 0 7 . 2 0 1 0  l   3 3 5 7   B U S _ S t a t u s   & = ~ ( 1 < < T W I _ C O N T R O L B I T ) ;   g e l o e s c h t  l   1 5 4 1 :   s p i s t a t u s   & =   ~ ( 1 < < S P I _ S E N D B I T ) ;   v o r   i f   s c h r e i b s t a t u s   v e r s c h o b e n  l   3 5 1 5 :   s h i f t   n u r   w e n n   s e n d b i t   u n d   k e i n   e r r b i t  l   3 4 1 9 :   B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ; 	 	 / /   T W I   w i e d e r   O N 	   g e l o e s c h t ,   i n   S t a t u s t a s k - A b r a g e   v e s c h o b e n  * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         0 3 . 0 7 . 2 0 1 0  S t a t u s t a s k   a b f r a g e n ,   b e i   S t a t u s   0   t i m e r 0   d e a k t i v i e r e n :   K e i n e   S t o e r u n g e n   d u c h   S P I - A u f r u f e   d e s   T i m e r s     * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         3 0 . 0 6 . 2 0 1 0       * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * * * * * * * *                                         2 7 . 0 6 . 2 0 1 0  D a t e n   v o n   E s t r i c h   a u c h   a m   A n f a n g   v o n   s p i b u f f e r   e i n g e s e t z t      * * * * * * * * * * * * * * * * * * * * * *   * * * * * * * * * * * * * * * * * * * * * *  T W I _ M a s t e r 3 2 _ 9 0 _ S R _ 1 6   V e r s i o n :   S R _ 1 6  2 6 . 0 3 . 2 0 1 0   * *  2 1 . 3 . 1 0  D A T A T A S K   a u f   C 0   g e s e t z t   ( W e b T x S t a r t D a t e n   > =   D A T A T A S K )   a n s t a t t   = = ,   V o r b e r e i t u n g   f u e r   u n t e r s c h e i d e n   v o n   S O L A R T A S K   u n d   D A T A T A S K   * *   5 . 2 . 1 0  B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ;   K o n t r o l l i e r e n ,   w i e   T W I   w i e d e r   e i n g e s c h a l t e t   w i r d  L i n i e   1 6 0 0  T i m e r   f u e r   T W I   z u r u e c k s e t z e n   f u e r   E E P R O M W R I T E T A S K   T e s t :    B U S _ S t a t u s   | = ( 1 < < T W I _ C O N T R O L B I T ) ;   i n   I S R :   e v e n t u e l l   z u   f r u e h .     	 	  i n   T a l k ( )   u n d   L i s t e n ( ) :  / /   T i m e r   f u e r   T W I - S c h l a u f e   z u r u e c k s e t z e n  	 	 T C N T 0   =   0 x 0 0 ;  	 	 S I G N A L _ C o u n t = 0 ;    * * *  1 0 . 1 2 . 0 9  s e t R e a d y T o S e n d   v o n   B e d i n g u n g   D a t e n [ 2 , 3 , 4 ]   n i c h t   N u l l   a b h � n g i g   g e m a c h t   2 5 . 1 0 . 0 9  c o u n t   d u r c h   S I G N A L _ _ c o u n t   e r s e t z t   2 . 2 . 0 9  l i n e   2 2 1 5  E i n g e f � g t :    i 2 c _ s t o p ( ) ;  T W I   z u t u e c k s e t z e n ;  T W B R   = 0 ;  d e l a y _ m s ( 1 0 ) ;   / / 	 T W I   n e u   s t a r t e n  i 2 c _ i n i t ( ) ;    1 . 2 .  l i n e   8 7 3  A n f a n g s w e r t   v o n   s t a r t d e l a y   i n   O D E R   e i n g e f u e g t :   K o n t r o l l e   l a e f t   a u c h   s o  s t a r t d e l a y   = =     P i n s   f u e r   T W I   z u e r s t   a l s   A u s g a e n g e   u n d   H I  E r s t   n a c h   A b l a u f   S t a r t d e l a y   z u   E i n g a e n g e n   g e m a c h t   l i n e   1 0 4 6  i 2 c _ s t o p   e i n g e f u e g t   l i n e   1 4 2 4  E n d e   v o n   i f   ( D C F 7 7 - e r f o l g = = 0 )   n a c h   2 1 4 4   v e r s c h o b e n :   A b f r a g e   d e r   S l a v e s   n u r   w e n n   U h r   g e l e s e n   w e r d e n   k a n n     l i n e   9 1 6  T W B R   = 0 ;   e r s e t z t   d u r c h  T W C R   = 0 ;  Q u e l l e :   f i l e : / / / U s e r s / s y s a d m i n / D o c u m e n t s / E l e k t r o n i k / A V R / I 2 C / T W I - E r r o r / T W I % 2 0 - % 2 0 A V R % 2 0 s t � r z t % 2 0 a b % 2 0 - % 2 0 m i k r o c o n t r o l l e r . n e t . w e b a r c h i v e   B U S _ S t a t u s :     3 1 . 1 . 0 9  l i n e   2 0 7 0    i 2 c _ s t o p ( )   e i n g e s e t z t     T W I _ M a s t e r 3 2 _ 9 0 _ S R _ 1 6  * * * * * * * * * * * * *  2 6 . 0 3 . 2 0 1 0                                                                                � �k"�k "  g� �j#�j C# C $%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdef$ �gg H   D a t u m _ V e r s i o n _ L o g _ a k t u a l i s i e r e n . a p p% �hh ,   L o g   a k t u a l i s i e r e n . a p p& �ii .   L o g   a k t u a l i s i e r e n . s c p t' �jj ( D a t u m   e i n s e t z e n . s c p t( �kk > D a t u m _ V e r s i o n _ a k t u a l i s i e r e n . a p p) �ll @ D a t u m _ V e r s i o n _ a k t u a l i s i e r e n . s c p t* �mm  F u n k t i o n e n . c+ �nn  F u n k t i o n e n . h, �oo 4 I O W a r r i o r P r o b e r _ P r e f i x . p c h- �pp 6 I O W a r r i o r W i n d o w C o n t r o l l e r . h. �qq 6 I O W a r r i o r W i n d o w C o n t r o l l e r . m/ �rr , L o g _ a k t u a l i s i e r e n . s c p t0 �ss  L o g f i l e . t x t1 �tt 4 M a c r o N a m e P a n e l C o n t r o l l e r . h2 �uu 4 M a c r o N a m e P a n e l C o n t r o l l e r . m3 �vv  M a k e f i l e4 �ww  T W I _ M a s t e r . c5 �xx  T W I _ M a s t e r . e e p6 �yy  T W I _ M a s t e r . e l f7 �zz  T W I _ M a s t e r . h8 �{{  T W I _ M a s t e r . h e x9 �||  T W I _ M a s t e r . l s s: �}}  T W I _ M a s t e r . l s t; �~~  T W I _ M a s t e r . m a p< �  T W I _ M a s t e r . o= ���  T W I _ M a s t e r . s y m> ��� ( T W I _ M a s t e r . x c o d e p r o j? ��� $ T W I _ M a s t e r 6 4 4 _ 2 0 _ 7@ ���   T W I _ M a s t e r _ l o g . cA ��� $ T W I _ M a s t e r _ l o g . t x tB ���  T W I _ l o g . t x tC ���  T e s t . s c p tD ��� J _ D a t u m _ V e r s i o n _ L o g _ a k t u a l i s i e r e n . s c p tE ��� 
 a d c . cF ��� 
 a d c . hG ���  a v r _ c o m p a t . hH ���  d a t u m . cI ���  d e f i n e s . hJ ���  d i s p l a y . cK ���  d i s p l a y . hL ���  e n c 2 8 j 6 0 . cM ���  e n c 2 8 j 6 0 . hN ��� 
 e r r . cO ��� 
 e r r . hP ���  i 2 c _ a v r . cQ ���  i 2 c _ a v r . hR ���  i 2 c m a s t e r . SS ���  i 2 c m a s t e r . hT ���   i p _ a r p _ u d p _ t c p . cU ���   i p _ a r p _ u d p _ t c p . hV ���  i p _ c o n f i g . hW ��� 
 l c d . cX ��� 
 l c d . hY ��� 
 n e t . hZ ���  r u n a v a r i c e . s h[ ���  s l a v e s . c\ ���  s l a v e s . h] ���  t w i _ s l . c^ ���  t w i m a s t e r . c_ ���  u a r t . c` ���  u a r t . ha ���  v e r s i o n . cb ���  w e b _ S P I . cc ���  w e b s r . cd ���  w e b s r . he ��� . w e b s r v _ h e l p _ f u n c t i o n s . cf ��� . w e b s r v _ h e l p _ f u n c t i o n s . h�  �  �  �  �  �  �  �  �  �
  �	  �  �  �   ascr  ��ޭ