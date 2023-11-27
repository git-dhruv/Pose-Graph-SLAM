"""
File:          helpers.py
Authors:       Binit Shah, Jerred Chen
Last Modified: Jerred Chen on 03/21/2022
"""

import numpy as np

import gtsam
from gtbook import driving
import plotly.graph_objects as go
from sklearn.neighbors import NearestNeighbors

def read_ply(*ply_fnames, as_animation=False):
    """Reads in one or more .ply files as numpy arrays.

    Args:
        ply_fnames (str):    filepath to the .ply file
        as_animation (bool): wraps each numpy array in a list,
                             useful for the animation viz
    """
    clouds = []
    for ply_fname in ply_fnames:
        np_cloud = driving.read_lidar_points(ply_fname)
        clouds.append([np_cloud] if as_animation else np_cloud)
    return clouds

# Access serialized result
def get_partial_map_results():
    serial = '22 serialization::archive 15 1 0\n0 0 0 60 0 2 33 gtsam::GenericValue<gtsam::Pose3> 1 0\n1 0 0 1 0\n2 1 0\n3 9.98274000054575672e-01 -5.72836990782530295e-02 1.29473998702203594e-02 5.75610990699765984e-02 9.98095500064065999e-01 -2.21839995307051008e-02 -1.16518999186280177e-02 2.28909995132142267e-02 9.99670100012095220e-01 0 0 -2.63946486492065674e+02 2.46730154676002348e+03 -1.93746526001808412e+01 1 2\n4\n5\n6 9.97722046037741395e-01 -6.63371763928519709e-02 1.22529346255159793e-02 6.65646665128786508e-02 9.97597272664248358e-01 -1.92058312588297073e-02 -1.09493732673290774e-02 1.99777162205955518e-02 9.99740504115084416e-01 -2.63795215223077207e+02 2.46732925164742119e+03 -1.93703463988572260e+01 2 2\n7\n8\n9 9.97017697562894933e-01 -7.62174469683654654e-02 1.21098328832173578e-02 7.64319807271601342e-02 9.96905474404627179e-01 -1.83756532855067316e-02 -1.06717528151193900e-02 1.92464518997864237e-02 9.99757851316880997e-01 -2.63631706190798866e+02 2.46733831901046915e+03 -1.93682285805464893e+01 3 2\n10\n11\n12 9.96189263861984475e-01 -8.64887104997760364e-02 1.12556674904428309e-02 8.66827178370322554e-02 9.96071925312076334e-01 -1.80793595832451845e-02 -9.64773316836071634e-03 1.89861569953400020e-02 9.99773234576664893e-01 -2.63463079582894352e+02 2.46734332858373728e+03 -1.93677887364932744e+01 4 2\n13\n14\n15 9.95060566497137811e-01 -9.87336782418039849e-02 1.03037037850362626e-02 9.89209492214948027e-02 9.94902028678682715e-01 -1.96121479094051102e-02 -8.31473531796469122e-03 2.05345476504154266e-02 9.99754605399749141e-01 -2.63269685488138407e+02 2.46733498510056233e+03 -1.93689504659714480e+01 5 2\n16\n17\n18 9.93247337494858495e-01 -1.15618143450267927e-01 9.60249930687938520e-03 1.15802766167007867e-01 9.93036381421576020e-01 -2.16449008006783412e-02 -7.03302659986241620e-03 2.26107555151135592e-02 9.99719642580110701e-01 -2.63026463424594567e+02 2.46732234607166720e+03 -1.93744164951667166e+01 6 2\n19\n20\n21 9.90609229535239888e-01 -1.36391428348889898e-01 9.52728508011656261e-03 1.36576693596218712e-01 9.90369240974334542e-01 -2.27070585436390977e-02 -6.33842023243324278e-03 2.37950450177072288e-02 9.99696800910246486e-01 -2.62750316757127507e+02 2.46732261091870168e+03 -1.93759529318741066e+01 7 2\n22\n23\n24 9.86404782946419312e-01 -1.63987860469007513e-01 1.06593774524720419e-02 1.64200097460877187e-01 9.86142878087565844e-01 -2.36767649916371087e-02 -6.62890493027270405e-03 2.51051614721410701e-02 9.99662874160834436e-01 -2.62479579450689357e+02 2.46737211613276668e+03 -1.93747654268966016e+01 8 2\n25\n26\n27 9.80672433914200892e-01 -1.95336937645743614e-01 1.11845733235309552e-02 1.95548310562305316e-01 9.80429342856712682e-01 -2.27859410874498222e-02 -6.51468523310929498e-03 2.45326831565326059e-02 9.99677838016863940e-01 -2.62233480654570428e+02 2.46743627498993055e+03 -1.93710454090801534e+01 9 2\n28\n29\n30 9.73230503831392757e-01 -2.29576669466556554e-01 1.08155340685508923e-02 2.29766215515544631e-01 9.72996487666332399e-01 -2.20308512342245154e-02 -5.46564405476586326e-03 2.39261530071498869e-02 9.99698824359599558e-01 -2.61979847872177686e+02 2.46749492290910666e+03 -1.93702075742590765e+01 10 2\n31\n32\n33 9.63988732046780150e-01 -2.65734778614650835e-01 1.05256092419535440e-02 2.65909276425678409e-01 9.63742014703310224e-01 -2.22176284926354738e-02 -4.23991170134186985e-03 2.42164105207641270e-02 9.99697785382600235e-01 -2.61730920655918737e+02 2.46756251033723584e+03 -1.93666630996403697e+01 11 2\n34\n35\n36 9.52193373210102334e-01 -3.05344306644775132e-01 9.62657482388063791e-03 3.05484937026662462e-01 9.51953719549543553e-01 -2.15199046249158893e-02 -2.59300943354723182e-03 2.34318914088513831e-02 9.99722109516292257e-01 -2.61466514291373755e+02 2.46765125092841299e+03 -1.93607807761990109e+01 12 2\n37\n38\n39 9.37596144561762102e-01 -3.47629675673075900e-01 8.19236368843010918e-03 3.47725689446901132e-01 9.37371861581715815e-01 -2.05152436730963728e-02 -5.47519517974971509e-04 2.20837130482162275e-02 9.99756011899407104e-01 -2.61189880464776479e+02 2.46776669458167316e+03 -1.93572134130140086e+01 13 2\n40\n41\n42 9.20768489630573561e-01 -3.90036271560651182e-01 7.55856460916411177e-03 3.90108770649471748e-01 9.20558448551636821e-01 -1.96806268198714482e-02 7.18122108069280812e-04 2.10699648119779918e-02 9.99777782495734013e-01 -2.60914396141427744e+02 2.46789092082485286e+03 -1.93556181908099703e+01 14 2\n43\n44\n45 9.01931087910968543e-01 -4.31823311040145374e-01 6.99837341927258237e-03 4.31875468470416413e-01 9.01729558386142882e-01 -1.91682152442894799e-02 1.96670630992592599e-03 2.03108335118466106e-02 9.99791816128224986e-01 -2.60590717314780534e+02 2.46800427425900216e+03 -1.93564422400221083e+01 15 2\n46\n47\n48 8.81709548583000102e-01 -4.71730416646038708e-01 7.66305924353570100e-03 4.71786890500040690e-01 8.81502831951263333e-01 -1.92334194481195292e-02 2.31804472234515202e-03 2.05736160807980586e-02 9.99785690274999639e-01 -2.60267117056977156e+02 2.46813706871216800e+03 -1.93550479883980664e+01 16 2\n49\n50\n51 8.60008960952231782e-01 -5.10205154370060865e-01 8.67894374553759146e-03 5.10272590777015145e-01 8.59789400326999353e-01 -1.95986580283866278e-02 2.53733642707576739e-03 2.12836414162432090e-02 9.99770294627998268e-01 -2.59945151702121166e+02 2.46829449287480975e+03 -1.93539286796875878e+01 17 2\n52\n53\n54 8.37376360375810425e-01 -5.46543412921593919e-01 9.54805761556353633e-03 5.46620713561889793e-01 8.37157377899630628e-01 -1.93224569547339464e-02 2.56739824575724423e-03 2.13993247949860084e-02 9.99767748456210303e-01 -2.59634382302679683e+02 2.46849402279531932e+03 -1.93536111132513611e+01 18 2\n55\n56\n57 8.14267183709157072e-01 -5.80414628277272748e-01 9.37279811832919009e-03 5.80483187679826762e-01 8.14071618085981163e-01 -1.80750285588229496e-02 2.86094514388172743e-03 2.01586417827310724e-02 9.99792737342592774e-01 -2.59315557707334392e+02 2.46871288207693487e+03 -1.93508309684831943e+01 19 2\n58\n59\n60 7.89961401746374525e-01 -6.13080639722362708e-01 9.65139766952993229e-03 6.13149410173626563e-01 7.89778480959412166e-01 -1.72565647403978700e-02 2.95726208893951114e-03 1.95497537559739219e-02 9.99804548508183344e-01 -2.58998589394247801e+02 2.46895236109208190e+03 -1.93467908262025361e+01 20 2\n61\n62\n63 7.65289066285535169e-01 -6.43607078200718830e-01 1.01296812487346958e-02 6.43680831802285636e-01 7.65119979919522297e-01 -1.63229993484383280e-02 2.75523829278439885e-03 1.90120770365914543e-02 9.99815494517531600e-01 -2.58686375698587085e+02 2.46917649010650530e+03 -1.93411062028886960e+01 21 2\n64\n65\n66 7.39455574190898246e-01 -6.73122217889876251e-01 1.05815929555094015e-02 6.73198352782470089e-01 7.39285069780024950e-01 -1.61740724857743247e-02 3.06437500999954182e-03 1.90834990450465189e-02 9.99813234138570373e-01 -2.58387610381933314e+02 2.46941772244763661e+03 -1.93402748613988109e+01 22 2\n67\n68\n69 7.12920997399224543e-01 -7.01166571799036187e-01 1.04463671836358615e-02 7.01234719697651609e-01 7.12754186460830552e-01 -1.58548031390710470e-02 3.67122635668462724e-03 1.86285551263552750e-02 9.99819770018365195e-01 -2.58098413640465651e+02 2.46966731974135473e+03 -1.93403264745175996e+01 23 2\n70\n71\n72 6.86574389193927992e-01 -7.26956896355482884e-01 1.22194710458575929e-02 7.27050821140502790e-01 6.86386965762717516e-01 -1.64339258679619335e-02 3.55952956254659272e-03 2.01672645583327488e-02 9.99790320356303219e-01 -2.57770595216489880e+02 2.46991822574951857e+03 -1.93368392011487451e+01 24 2\n73\n74\n75 6.59942779663973744e-01 -7.51157481052009857e-01 1.54273379894104071e-02 7.51310957695496606e-01 6.59725411826690866e-01 -1.71540970687149887e-02 2.70767998718500057e-03 2.29114239433578747e-02 9.99733868892421107e-01 -2.57447213520264654e+02 2.47018721870037507e+03 -1.93312498683461520e+01 25 2\n76\n77\n78 6.32619038218620422e-01 -7.74202414247752047e-01 2.00950437857828831e-02 7.74462845771556108e-01 6.32384366150158472e-01 -1.72438736294911882e-02 6.42514628520111670e-04 2.64716388343228144e-02 9.99649395038586519e-01 -2.57116399561353205e+02 2.47049644475479045e+03 -1.93252015235422974e+01 26 2\n79\n80\n81 6.04666362446230843e-01 -7.96080886139986399e-01 2.51763654459621726e-02 7.96477439811150045e-01 6.04422587733477901e-01 -1.72354436817384778e-02 -1.49630017046328378e-03 3.04740693954736012e-02 9.99534474477450918e-01 -2.56793625465424782e+02 2.47082628143632655e+03 -1.93181684296562821e+01 27 2\n82\n83\n84 5.76569718835345801e-01 -8.16484740773522844e-01 3.03325556881431611e-02 8.17040656568825963e-01 5.76323853392624375e-01 -1.71877495246483478e-02 -3.44778476407197731e-03 3.46928344513909304e-02 9.99392111989374499e-01 -2.56479255303116872e+02 2.47116394633553045e+03 -1.93122510815621311e+01 28 2\n85\n86\n87 5.47937885937136548e-01 -8.35752397893310350e-01 3.58055735713534232e-02 8.36499122117281835e-01 5.47717853041031599e-01 -1.65653061126002886e-02 -5.76680334491560132e-03 3.90280551100770229e-02 9.99221511168638998e-01 -2.56176727416256881e+02 2.47151903343603499e+03 -1.93043299182845196e+01 29 2\n88\n89\n90 5.18610022182193653e-01 -8.54044208199407517e-01 4.06469170009442424e-02 8.54979352514257274e-01 5.18412896793743694e-01 -1.60752114040433672e-02 -7.34289179190437263e-03 4.30890041141028801e-02 9.99044289885699599e-01 -2.55891391617187537e+02 2.47188019685780955e+03 -1.92992643341323920e+01 30 2\n91\n92\n93 4.89103628617479691e-01 -8.71058961776080776e-01 4.50994690793828437e-02 8.72189867766462013e-01 4.88897845446072787e-01 -1.62409511167039486e-02 -7.90215552070776261e-03 4.72787699101114497e-02 9.98850513011106056e-01 -2.55614088643970945e+02 2.47224511525708976e+03 -1.92969039484370910e+01 31 2\n94\n95\n96 4.60289934043867421e-01 -8.86400674960695079e-01 4.92651659447860824e-02 8.87737287090485605e-01 4.60030841119834832e-01 -1.71514295983579632e-02 -7.46040650948269143e-03 5.16291153136970923e-02 9.98638498294188937e-01 -2.55344863520023381e+02 2.47262469655120594e+03 -1.92998205788634571e+01 32 2\n97\n98\n99 4.32261020578599831e-01 -9.00183242646056425e-01 5.31090978649664189e-02 9.01718939501415018e-01 4.31973601431559884e-01 -1.73723636894246684e-02 -7.30336842445586647e-03 5.53988336642389709e-02 9.98437631265436387e-01 -2.55080459307733349e+02 2.47301558682169889e+03 -1.93026256232639319e+01 33 2\n100\n101\n102 4.06799125409961160e-01 -9.11713219350442361e-01 5.73891431875009281e-02 9.13486748091071332e-01 4.06498449736498180e-01 -1.73496003783809652e-02 -7.51068973397881704e-03 5.94819812979026918e-02 9.98201160552938394e-01 -2.54825853501933665e+02 2.47344556511905057e+03 -1.93077640215652870e+01 34 2\n103\n104\n105 3.82159700324761598e-01 -9.21974411516522485e-01 6.25874147485180471e-02 9.24057575027601330e-01 3.81886209052843706e-01 -1.67498829495461229e-02 -8.45826025525428862e-03 6.42354609474161120e-02 9.97898961257584594e-01 -2.54573240702603243e+02 2.47392225066958054e+03 -1.93103011886473546e+01 35 2\n106\n107\n108 3.56911908151500423e-01 -9.31688160868737958e-01 6.76098901429035143e-02 9.34088780605001556e-01 3.56700418332516145e-01 -1.55884161715030729e-02 -9.59288769745270629e-03 6.87172859649289869e-02 9.97590088465135527e-01 -2.54326277585368786e+02 2.47440519345733355e+03 -1.93105216537000537e+01 36 2\n109\n110\n111 3.31717816537257570e-01 -9.40687067868972049e-01 7.12121272276156209e-02 9.43320532358223440e-01 3.31588963049583685e-01 -1.39703265733996194e-02 -1.04714055100456963e-02 7.18100215661923169e-02 9.97363396139013481e-01 -2.54106228664625064e+02 2.47490390151820429e+03 -1.93103479457468090e+01 37 2\n112\n113\n114 3.08202276452176405e-01 -9.48408763966912982e-01 7.43788259446212657e-02 9.51266849958616989e-01 3.08074458776490312e-01 -1.34738720808070603e-02 -1.01354349783882962e-02 7.49067420057787925e-02 9.97139070766218771e-01 -2.53904237567641047e+02 2.47541251951192817e+03 -1.93143811021678218e+01 38 2\n115\n116\n117 2.85058969450344313e-01 -9.55491631915476813e-01 7.60076430621252769e-02 9.58468954102125736e-01 2.84882880313229003e-01 -1.33807972962706277e-02 -8.86799440091301343e-03 7.66652339069292393e-02 9.97017489352088782e-01 -2.53723942740367562e+02 2.47592628698171256e+03 -1.93175838709523298e+01 39 2\n118\n119\n120 2.64269567912186087e-01 -9.61586551341131912e-01 7.42491500545827687e-02 9.64421412238868014e-01 2.64058593212577963e-01 -1.28232669491964695e-02 -7.27540409493652139e-03 7.49962199314879036e-02 9.97157314058260469e-01 -2.53583606071205224e+02 2.47644774765496049e+03 -1.93177269871153356e+01 40 2\n121\n122\n123 2.44685534083971867e-01 -9.66942002060939942e-01 7.17787614052804585e-02 9.69585935146124234e-01 2.44442646262113372e-01 -1.22859376793093694e-02 -5.66596124590505994e-03 7.26018184540734302e-02 9.97344948515289431e-01 -2.53456502973469952e+02 2.47697301307717134e+03 -1.93190639066120511e+01 41 2\n124\n125\n126 2.29623633195566446e-01 -9.70872705155037985e-01 6.84047804722809349e-02 9.73269071430429622e-01 2.29378806180218331e-01 -1.15202031994907812e-02 -4.50591686326682131e-03 6.92215171880101454e-02 9.97591174660269253e-01 -2.53307413810690434e+02 2.47751513262178787e+03 -1.93228844181541248e+01 42 2\n127\n128\n129 2.15476704942813374e-01 -9.74335450105783152e-01 6.51172544154237642e-02 9.76502349727602970e-01 2.15241490846483641e-01 -1.06910618801127084e-02 -3.59921590139080207e-03 6.58907752571959160e-02 9.97820387078529603e-01 -2.53160726511761936e+02 2.47806049631340147e+03 -1.93237088365589429e+01 43 2\n130\n131\n132 2.00498397977139614e-01 -9.77692784818526950e-01 6.25879177705138956e-02 9.79690098947163968e-01 2.00269155027309020e-01 -9.98062017188489428e-03 -2.77641144147707181e-03 6.33178096431911697e-02 9.97989589129137711e-01 -2.53019862271919948e+02 2.47860439701203677e+03 -1.93280426599268829e+01 44 2\n133\n134\n135 1.84975833136020923e-01 -9.80874459287549683e-01 6.05745227092038749e-02 9.82741282895489232e-01 1.84741537300863790e-01 -9.49592007730778835e-03 -1.87628817003125540e-03 6.12855472236667406e-02 9.98118547434547199e-01 -2.52890465676880410e+02 2.47915444776834374e+03 -1.93296142468695180e+01 45 2\n136\n137\n138 1.69065241002560956e-01 -9.83816448973154456e-01 5.93479194500950435e-02 9.85604247109150289e-01 1.68826006566895892e-01 -9.06006603472863346e-03 -1.10599429752328304e-03 6.00252504599626674e-02 9.98196283088894520e-01 -2.52769082876610895e+02 2.47970197512711457e+03 -1.93312991510183956e+01 46 2\n139\n140\n141 1.53853174161364487e-01 -9.86413383433462521e-01 5.76009922855182169e-02 9.88093617605166230e-01 1.53619067678910176e-01 -8.49835883404828976e-03 -4.65680720218615378e-04 5.82226185182919825e-02 9.98303552694765450e-01 -2.52662284556117072e+02 2.48027102232079142e+03 -1.93379498616674681e+01 47 2\n142\n143\n144 1.40662821950315098e-01 -9.88486561077116188e-01 5.57523597649964148e-02 9.90057535602984040e-01 1.40425533376134559e-01 -8.17208548952552336e-03 2.48976224423134898e-04 5.63474982408015343e-02 9.98411223365149536e-01 -2.52552810443731005e+02 2.48084497169391170e+03 -1.93414713648721701e+01 48 2\n145\n146\n147 1.29219339126742383e-01 -9.90126324408211578e-01 5.43347093301148679e-02 9.91615731831171798e-01 1.28981744659148023e-01 -7.87319116280524485e-03 7.87301991341183855e-04 5.48964664384921225e-02 9.98491778442263445e-01 -2.52444155704591651e+02 2.48142637024710666e+03 -1.93430644827261133e+01 49 2\n148\n149\n150 1.17972264253392856e-01 -9.91512810799493716e-01 5.46344892523927469e-02 9.93016412226536138e-01 1.17739064419841083e-01 -7.48030532480735161e-03 9.84238054767796761e-04 5.51353580028292878e-02 9.98478440960045455e-01 -2.52345644470522899e+02 2.48202888423790182e+03 -1.93439958461626027e+01 50 2\n151\n152\n153 1.07251835268923498e-01 -9.92696426778126528e-01 5.52348134951376052e-02 9.94231167008489680e-01 1.07018893023987552e-01 -7.16800490587437677e-03 1.20451680995971267e-03 5.56848993546287183e-02 9.98447702492109901e-01 -2.52251864853419107e+02 2.48263766117185105e+03 -1.93447754712891360e+01 51 2\n154\n155\n156 9.73843703130690408e-02 -9.93699772684001448e-01 5.54714585615510222e-02 9.95245864031173988e-01 9.71542369880733780e-02 -6.83824345726073094e-03 1.40590573527457283e-03 5.58736220113724424e-02 9.98436895993455420e-01 -2.52176653620832866e+02 2.48326672467522758e+03 -1.93451296048625210e+01 52 2\n157\n158\n159 8.83875552878605059e-02 -9.94550900836665197e-01 5.52827481559034772e-02 9.96084640359002460e-01 8.81539156561620824e-02 -6.65684646345308766e-03 1.74721342058021720e-03 5.56546226852372428e-02 9.98448588414888816e-01 -2.52111648945731417e+02 2.48389331743329740e+03 -1.93484431711160951e+01 53 2\n160\n161\n162 8.21428134029617008e-02 -9.95260293574678601e-01 5.20532685654413810e-02 9.96617908696854848e-01 8.19096028087409750e-02 -6.60289647782103233e-03 2.30796927339149823e-03 5.24195439311731723e-02 9.98622520365195165e-01 -2.52053332487284166e+02 2.48456875428745479e+03 -1.93548062868253119e+01 54 2\n163\n164\n165 7.76022016803988857e-02 -9.95807919724998936e-01 4.84202626570789524e-02 9.96980021669357508e-01 7.73661866973452128e-02 -6.73400011542505447e-03 2.95971044838235723e-03 4.87965513758481031e-02 9.98804390347049931e-01 -2.51999278551731379e+02 2.48524895276146844e+03 -1.93608519768120360e+01 55 2\n166\n167\n168 7.39203494678461609e-02 -9.96193324621982401e-01 4.62025804648372659e-02 9.97259415850187181e-01 7.36977779073845207e-02 -6.50632301180334872e-03 3.07655871242617238e-03 4.65568515843890890e-02 9.98910940897985489e-01 -2.51930921685373875e+02 2.48590898506492385e+03 -1.93624175722222489e+01 56 2\n169\n170\n171 7.27185331754320319e-02 -9.96290634321224555e-01 4.60111228409184753e-02 9.97348511775591806e-01 7.25101734996553005e-02 -6.18530577570035496e-03 2.82611832060447208e-03 4.63388547251890187e-02 9.98921817304365467e-01 -2.51862497882755662e+02 2.48660427392939346e+03 -1.93631600243962900e+01 57 2\n172\n173\n174 7.23660121180367266e-02 -9.96288507308968896e-01 4.66090873398682659e-02 9.97375042362502295e-01 7.21700336710773410e-02 -5.87778204678805673e-03 2.49221788502064718e-03 4.69120355693981791e-02 9.98895952167897616e-01 -2.51792590631067299e+02 2.48733010496013094e+03 -1.93686321981784957e+01 58 2\n175\n176\n177 7.23781697156478526e-02 -9.96221545250658203e-01 4.80007276555304424e-02 9.97374277604991755e-01 7.21761938823353016e-02 -5.93166233509504100e-03 2.44477057834880307e-03 4.83039573995453775e-02 9.98829727373501730e-01 -2.51725647961007468e+02 2.48806001421560632e+03 -1.93786797557644306e+01 59 2\n178\n179\n180 7.16751499585385476e-02 -9.96254882466123703e-01 4.83623672837365029e-02 9.97425425450651160e-01 7.14796769240499952e-02 -5.76312943616315645e-03 2.28464999720698466e-03 4.86508713763659784e-02 9.98813269123599135e-01 -2.51657633808848686e+02 2.48870514300924606e+03 -1.93794338854928583e+01\n'
    return serial

def get_identity_results():
    serial = '22 serialization::archive 15 1 0\n0 0 0 50 0 2 33 gtsam::GenericValue<gtsam::Pose3> 1 0\n1 0 0 1 0\n2 1 0\n3 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0 0 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1 2\n4\n5\n6 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 2 2\n7\n8\n9 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 3 2\n10\n11\n12 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 4 2\n13\n14\n15 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 5 2\n16\n17\n18 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 6 2\n19\n20\n21 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 7 2\n22\n23\n24 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 8 2\n25\n26\n27 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 9 2\n28\n29\n30 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 10 2\n31\n32\n33 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 11 2\n34\n35\n36 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 12 2\n37\n38\n39 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 13 2\n40\n41\n42 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 14 2\n43\n44\n45 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 15 2\n46\n47\n48 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 16 2\n49\n50\n51 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 17 2\n52\n53\n54 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 18 2\n55\n56\n57 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 19 2\n58\n59\n60 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 20 2\n61\n62\n63 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 21 2\n64\n65\n66 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 22 2\n67\n68\n69 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 23 2\n70\n71\n72 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 24 2\n73\n74\n75 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 25 2\n76\n77\n78 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 26 2\n79\n80\n81 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 27 2\n82\n83\n84 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 28 2\n85\n86\n87 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 29 2\n88\n89\n90 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 30 2\n91\n92\n93 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 31 2\n94\n95\n96 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 32 2\n97\n98\n99 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 33 2\n100\n101\n102 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 34 2\n103\n104\n105 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 35 2\n106\n107\n108 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 36 2\n109\n110\n111 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 37 2\n112\n113\n114 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 38 2\n115\n116\n117 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 39 2\n118\n119\n120 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 40 2\n121\n122\n123 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 41 2\n124\n125\n126 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 42 2\n127\n128\n129 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 43 2\n130\n131\n132 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 44 2\n133\n134\n135 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 45 2\n136\n137\n138 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 46 2\n139\n140\n141 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 47 2\n142\n143\n144 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 48 2\n145\n146\n147 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 49 2\n148\n149\n150 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 1.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00\n'
    return serial

def assign_closest_pairs_KDTree(clouda, cloudb):
    """Returns a rearranged version of cloudb,
    where index-wise corresponding points in
    clouda and rearranged cloudb are closest
    euclidean distance.

    Note: Don't modify clouda or cloudb here.

    Args:
        clouda (a numpy array of shape (3, n)): point cloud A
        cloudb (a numpy array of shape (3, m)): point cloud B

    Returns:
        a numpy array of shape (3, n), where each 3D point is a
        point from cloudb closest to the point at the same index
        in clouda
    """

    # KD Tree Implementation
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(cloudb.transpose())
    _, indices = nbrs.kneighbors(clouda.transpose())
    return cloudb.transpose()[indices.T][0].transpose()
