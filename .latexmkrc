# Выбор движка по целевому файлу:
#   * файлы с \usepackage{fontspec} (старый ultrasonic-navigation.tex) -> xelatex
#   * остальные (диплом main.tex по ГОСТ-шаблону) -> pdflatex + bibtex
my $use_xe = 0;
foreach my $arg (@ARGV) {
    next if $arg =~ /^-/;                 # пропускаем опции
    my $f = $arg;
    $f .= '.tex' unless ($f =~ /\.tex$/ || -e $f);
    next unless -e $f;
    if (open(my $fh, '<', $f)) {
        local $/;
        my $content = <$fh>;
        close($fh);
        $use_xe = 1 if $content =~ /\\usepackage(\[[^\]]*\])?\{fontspec\}/;
    }
}

if ($use_xe) {
    $pdf_mode = 5;   # xelatex
    $xelatex  = 'xelatex -interaction=nonstopmode -halt-on-error %O %S';
} else {
    $pdf_mode = 1;   # pdflatex
    $pdflatex = 'pdflatex -interaction=nonstopmode -file-line-error %O %S';
}

$bibtex_use = 2;     # запускать bibtex по мере необходимости
