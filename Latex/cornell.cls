%%
%% This is file `cornell.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% cornell.dtx  (with options: `class')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from cornell.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file cornell.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
%% Document class `cornell' to use with LaTeX 2e.
%% Based on `crnlphd' class.  The following is from the file `crnlphd.dtx':
%%   Based loosely on old cuphdthesis style file.
%%   Found to give good results, according to the thesis secretary.
%%   File: crnlphd.dtx Copyright (C) 1994 Sergio Gelato
%%   Made available without any warranty, express or implied.
%% Modifications
%%   03/26/98 Mike Heisler
%%              Conform to Eric's final version. Margins are subject to
%%              The whim of the printer.
%%   10/10/97 Mike Heisler and Eric Dahl
%% Altered footnotesize and changed the Acknowledgement and
%% Biographical titles per thesis secretary guideline that
%% they must match the rest of the document, i.e. flush left
%% and capitalized rather than all caps
%%   05/21/97 Mike Heisler
%%            Added a 'papers' option to handle bibliographis as sections
%%            at the end of each chapter.  Reduced top margin.
%%
%% $Id$
%% \CharacterTable
%%  {Upper-case    \A\B\C\D\E\F\G\H\I\J\K\L\M\N\O\P\Q\R\S\T\U\V\W\X\Y\Z
%%   Lower-case    \a\b\c\d\e\f\g\h\i\j\k\l\m\n\o\p\q\r\s\t\u\v\w\x\y\z
%%   Digits        \0\1\2\3\4\5\6\7\8\9
%%   Exclamation   \!     Double quote  \"     Hash (number) \#
%%   Dollar        \$     Percent       \%     Ampersand     \&
%%   Acute accent  \'     Left paren    \(     Right paren   \)
%%   Asterisk      \*     Plus          \+     Comma         \,
%%   Minus         \-     Point         \.     Solidus       \/
%%   Colon         \:     Semicolon     \;     Less than     \<
%%   Equals        \=     Greater than  \>     Question mark \?
%%   Commercial at \@     Left bracket  \[     Backslash     \\
%%   Right bracket \]     Circumflex    \^     Underscore    \_
%%   Grave accent  \`     Left brace    \{     Vertical bar  \|
%%   Right brace   \}     Tilde         \~}
%%
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{cornell}
[1998/03/26 v2.0
  Cornell University thesis class]
\newif\ifphd
\DeclareOption{phd}{\phdtrue}
\DeclareOption{master}{\phdfalse}
\ExecuteOptions{phd}
\newif\ifpapers
\DeclareOption{papers}{\paperstrue}
\DeclareOption{10pt}{%
  \OptionNotUsed%
  \ClassWarningNoLine{cornell}{Font size 10pt not allowed; using 12pt}%
}
\DeclareOption{11pt}{%
  \OptionNotUsed%
  \ClassWarningNoLine{cornell}{Font size 11pt not allowed; using 12pt}%
}
\newcommand{\prelim@contents}[1]{}
\DeclareOption{tocprelim}{%
  \renewcommand{\prelim@contents}[1]{\addcontentsline{toc}{section}{#1}}
}
\DeclareOption*{%
  \PassOptionsToClass{\CurrentOption}{report}%
}
\ProcessOptions

\LoadClass[12pt]{report}[1994/06/01]
\renewcommand{\defaultscriptratio}{.75}
\renewcommand{\defaultscriptscriptratio}{.6}
\setlength\headsep{25\p@}
\if@twocolumn
\else
  \setlength\oddsidemargin{0in}
  \setlength\evensidemargin{0in}
  \setlength\textwidth{6.5in}
  \setlength\topmargin{-0.5in}
  \setlength\headheight{0.2in}
  \setlength\headsep{0.35in}
  \setlength\textheight{8.85in}
  \setlength\footskip{0.5in}
  \setlength\marginparwidth{0.85in}
  \setlength\marginparsep{0.125in}
\fi
\if@compatibility
  \setlength\topmargin{0\p@}
\else
\fi
\newcommand{\ps@cornell}{%
\def\@oddhead{{\slshape\rightmark}\hfil\thepage}%
\def\@oddfoot{}%
\def\@evenhead{\thepage\hfil{\slshape\leftmark}}%
\def\@evenfoot{}%
\def\chaptermark##1{}%
\def\sectionmark##1{}%
}
\newcommand{\ps@cornellc}{%
\def\@oddhead{\hfil\thepage\hfil}%
\def\@oddfoot{}%
\def\@evenhead{\@oddhead}%
\def\@evenfoot{}%
\def\chaptermark##1{}%
\def\sectionmark##1{}%
}
\raggedbottom
\newcommand{\@prolformat}[1]{{\begin{flushleft} \Huge #1 \end{flushleft}} }
\newcommand{\ackname}{\@prolformat{Acknowledgements}}
\newcommand{\bioname}{\@prolformat{Biographical Sketch}}
\renewcommand{\contentsname}{\@prolformat{Table of Contents}}
\renewcommand{\listtablename}{\@prolformat{List of Tables}}
\renewcommand{\listfigurename}{\@prolformat{List of Figures}}
\renewcommand{\l@figure}{\@dottedtocline{1}{1.5em}{2.6em}}
\let\l@table\l@figure

\renewenvironment{thebibliography}[1]
     {\ifpapers \clearpage \section*{\bibname%
                \@mkboth{\uppercase{\bibname}}{\uppercase{\bibname}}}%
         \else%
                \chapter*{\bibname%
                \@mkboth{\uppercase{\bibname}}{\uppercase{\bibname}}}%
      \fi%
      \list{\@biblabel{\arabic{enumiv}}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\arabic{enumiv}}}%
      \sloppy\clubpenalty4000\widowpenalty4000%
      \sfcode`\.\@m\relax
      \ifpapers%
            \addcontentsline{toc}{section}{\bibname}%
         \else%
              \addcontentsline{toc}{chapter}{\bibname}%
      \fi
      \singlespacing}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \normalspacing
      \endlist}

\newcommand{\@titlesize}{\Large}
\newcommand{\@subtitlesize}{\large}
\newcommand{\@authorsize}{\normalsize} % No Pharaoh he.
\newcommand{\@conferralmonth}{January}
\newcommand{\@conferralyear}{}

\newcommand{\titlesize}[1]{\renewcommand{\@titlesize}{#1}}
\newcommand{\subtitlesize}[1]{\renewcommand{\@subtitlesize}{#1}}
\newcommand{\authorsize}[1]{\renewcommand{\@authorsize}{#1}}
\newcommand{\conferraldate}[2]{%
  \renewcommand{\@conferralmonth}{#1}
  \renewcommand{\@conferralyear}{#2}
}
\newcommand{\@degreefield}{Master of Science} %CHANGED
\newcommand{\degreefield}[1]{\renewcommand{\@degreefield}{#1}}  %CHANGED

\renewcommand{\maketitle}{%
  \begin{titlepage}%
    \vspace*{-0.24in}
    \begin{center}%
      \@titlesize \@title
    \end{center}
    \vfill
    %\begin{center}
      %\@subtitlesize
      %A \ifphd Dissertation \else Thesis \fi \\ %CHANGED
      %Presented to the Faculty of the Graduate School \\
      %of Cornell University \\
      %in Partial Fulfillment of the Requirements for the Degree of \\
      %\ifphd Doctor of Philosophy \else \@degreefield \fi %CHANGED
    %\end{center}
    \vfill
    \begin{center}
      \@authorsize
      by \\
      \@author \\
      \@conferralmonth\ \@conferralyear
    \end{center}
    \vspace*{0.26in}
  \end{titlepage}
}

\renewenvironment{titlepage}{%
  \typeout{Title page}
  \pagestyle{empty}
  \doublespacing
}{%
  \normalspacing\clearpage
}


\newenvironment{copyrightpage}{%
  \typeout{Copyleft page}
  \pagestyle{empty}
  \null\vfil
  \begin{center}
}{%
  \end{center}
  \vfil\null
  \clearpage
}

\newcommand{\makecopyright}{%
  \begin{copyrightpage}
  \copyright\ \@copyrightholder\ \@copyrightyear \\
  ALL RIGHTS RESERVED \\[.5in]
  \end{copyrightpage}
}
\newcommand{\@copyrightholder}{\@author}
\newcommand{\@copyrightyear}{\@conferralyear}

\newcommand{\copyrightholder}[1]{\renewcommand{\@copyrightholder}{#1}}
\newcommand{\copyrightyear}[1]{\renewcommand{\@copyrightyear}{#1}}


\newlength{\@abhdskip}
\renewenvironment{abstract}{%
  \typeout{Abstract}
  \singlespacing
  \ifphd %CHANGED
  \setlength{\@abhdskip}{.25in}
  \addtolength{\@abhdskip}{-\topmargin}
  \vspace*{\@abhdskip}
    \begin{center}
      \@abstracttitle \\
      \ \\
      \@author \\
      \@conferralyear
    \end{center}
    \vspace{.25in}
  \else %CHANGED
    \chapter*{%
      \begin{center}
        \normalsize
        ABSTRACT
      \end{center}
    }
    \par
  \fi %CHANGED
  \pagestyle{empty}
  \thispagestyle{empty}
}{%
  \clearpage
}

\newcommand{\@abstracttitle}{\@title}
\newcommand{\abstracttitle}[1]{\renewcommand{\@abstracttitle}{#1}}


\newenvironment{biosketch}{%
  \typeout{Vita}
  \pagestyle{plain}
  \pagenumbering{roman}
  \setcounter{page}{3}
  \prelim@contents{Biographical Sketch}
  \chapter*{%
\bioname
  }
  \par
}{%
  \clearpage
}

\newenvironment{dedication}{%
  \prelim@contents{Dedication}
  \null\vfil
  \begin{center}
}{
  \end{center}
  \vfil\null
  \clearpage
}

\newenvironment{acknowledgements}{%
  \prelim@contents{Acknowledgments}
  \chapter*{%
\ackname
  }
  \par
}{%
  \cleardoublepage
}

\newcommand{\contentspage}{%
  \singlespacing
  \label{toc}
  \prelim@contents{\contentsname}
  \tableofcontents
  \normalspacing
  \clearpage
}

\newcommand{\tablelistpage}{%
  \singlespacing
  \label{lot}
  \prelim@contents{\listtablename}
  \listoftables
  \normalspacing
  \clearpage
}

\newcommand{\figurelistpage}{%
  \singlespacing
  \label{lof}
  \prelim@contents{\listfigurename}
  \listoffigures
  \normalspacing
  \clearpage
}
\newcommand{\abbrlist}{%
  \singlespacing
  \prelim@contents{List of Abbreviations}
  \chapter*{List of Abbreviations}
}

\newcommand{\symlist}{%
  \singlespacing
  \prelim@contents{List of Symbols}
  \chapter*{List of Symbols}
}

\newcommand{\preface}{%
  \singlespacing
  \prelim@contents{Preface}
  \chapter*{Preface}
}

\newcommand{\singlespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.0}\tiny\CS}
\newcommand{\singlespacingplus}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.25}\tiny\CS}
\newcommand{\doublespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.75}\tiny\CS}
\newcommand{\realdoublespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{2}\tiny\CS}
\newcommand{\normalspacing}{\doublespacing}
\newcommand{\footnotespacing}{\singlespacing}
\newcommand{\changespacing}[2]{%
  \renewcommand{#1}{%
    \let\CS=\@currsize\renewcommand{\baselinestretch}{#2}\tiny\CS}%
}
\newcommand{\changenormalspacing}[1]{\renewcommand{\normalspacing}{#1}}
\normalspacing

\endinput
%%
%% End of file `cornell.cls'.
